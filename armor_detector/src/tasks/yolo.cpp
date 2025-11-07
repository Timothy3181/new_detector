#include <armor_detector/tasks/yolo.hpp>

namespace pka {

YOLO::YOLO(const std::string& model_path, const LightParams& light_params, const int threshold, const bool fix, const float conf, const float nms, const Color color) {
    // init core
    this->core = ov::Core();

    // model setting
    auto model = core.read_model(model_path);
    auto ppp = new ov::preprocess::PrePostProcessor(model);
    ppp->input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::RGB);
    ppp->input().preprocess().convert_element_type(ov::element::f32).scale({255., 255., 255.});
    ppp->input().model().set_layout("NCHW");
    ppp->output().tensor().set_element_type(ov::element::f32);
    model = ppp->build();
    this->compiled_model = core.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

    // init tradition check
    this->light_params = light_params;
    this->threshold = threshold;
    this->fix_points = fix;
    this->conf_threshold = conf;
    this->nms_threshold = nms;
    this->detect_color = color;
}

bool YOLO::fixPoints(Armor& armor, cv::Mat& image) {
    // cal new four points
    auto tl = armor.points[0];
    auto tr = armor.points[1];
    auto br = armor.points[2];
    auto bl = armor.points[3];
    auto lt2b = bl - tl;
    auto rt2b = br - tr;
    auto tl1 = (tl + bl) / 2 - lt2b;
    auto bl1 = (tl + bl) / 2 + lt2b;
    auto br1 = (tr + br) / 2 + rt2b;
    auto tr1 = (tr + br) / 2 - rt2b;
    auto tl2tr = tr1 - tl1;
    auto bl2br = br1 - bl1;
    auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
    auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
    auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
    auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;

    // get roi
    std::vector<cv::Point2f> n_points = {tl2, tr2, bl2, br2};
    auto rrect = cv::minAreaRect(n_points);
    cv::Rect rect = rrect.boundingRect();

    // check if roi is available
    if (rect.x < 0 || rect.y < 0 || rect.x + rect.width > image.cols || rect.y + rect.height > image.rows) {
        return false;
    }

    // get roi
    cv::Mat roi = image(rect);

    // to binary
    cv::Mat gray_roi;
    cv::cvtColor(roi, gray_roi, cv::COLOR_RGB2GRAY);
    cv::threshold(gray_roi, this->binary_img, this->threshold, 255, cv::THRESH_BINARY);

    // find lights
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(this->binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<Light> lights;
    for (auto& contour : contours) {
        // init light
        if (contour.size() < 6) continue;
        auto light = Light(contour, cv::Point2f(rect.x, rect.y));

        // check light
        float ratio = light.height / light.width;
        bool ratio_ok = this->light_params.min_ratio < ratio && ratio < this->light_params.max_ratio;
        bool angle_ok = light.tilt_angle < light_params.max_angle;
        if (ratio_ok && angle_ok) {
            lights.emplace_back(light);
        }
    }

    float min_left_light_dist = std::numeric_limits<float>::max();
    float min_right_light_dist = std::numeric_limits<float>::max();
    Light selected_left;
    Light selected_right;
    for (auto& light : lights)  {
        // select available light
        auto left_dist_error = cv::norm(light.center - armor.left.center);
        // left light
        if (left_dist_error < min_left_light_dist) {
            selected_left = light;
            min_left_light_dist = left_dist_error;
        }
        // right light
        auto right_dist_error = cv::norm(light.center - armor.right.center);
        if (right_dist_error < min_right_light_dist) {
            selected_right = light;
            min_right_light_dist = right_dist_error;
        }
    }

    // fix key points
    std::vector<cv::Point2f> fix_points = {
        (selected_left.top + cv::Point2f(rect.x, rect.y)), 
        (selected_right.top + cv::Point2f(rect.x, rect.y)), 
        (selected_right.bottom + cv::Point2f(rect.x, rect.y)), 
        (selected_left.bottom + cv::Point2f(rect.x, rect.y))
    };
    armor.points = fix_points;
    
    return true;
}

std::vector<Armor> YOLO::detect(cv::Mat& image) {
    // init armors
    std::vector<Armor> armors;

    // fix image
    double scale = 0;
    cv::Mat fixed_img = resizeImg(image, scale);

    // set input tensor
    ov::Tensor input_tensor(
        this->compiled_model.input().get_element_type(),
        this->compiled_model.input().get_shape(),
        static_cast<uchar*>(fixed_img.data)
    );

    // inference
    auto infer_request = this->compiled_model.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // set output tensor
    ov::Tensor output_tensor = infer_request.get_output_tensor();
    ov::Shape output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

    // process data
    std::vector<float> confs;
    std::vector<cv::Rect> boxes;
    std::vector<int> color_ids;
    std::vector<int> symbol_ids;
    std::vector<std::vector<cv::Point2f>> all_key_points;
    for (int i = 0; i < output.rows; i++) {
        // confidence
        auto conf = output.at<float>(i, 8);
        conf = sigmoid(conf);
        if (conf < this->conf_threshold) continue;

        // color and symbols
        cv::Mat color_vec = output.row(i).colRange(9, 13);
        cv::Mat symbol_vec = output.row(i).colRange(13, 22);
        cv::Point color_id, symbol_id;
        int _color_id, _symbol_id;
        double color_conf, symbol_conf;
        cv::minMaxLoc(color_vec, NULL, &color_conf, NULL, &color_id);
        cv::minMaxLoc(symbol_vec, NULL, &symbol_conf, NULL, &symbol_id);
        _color_id = color_id.x;
        _symbol_id = symbol_id.x;

        // key points
        // 需要恢复比例
        std::vector<cv::Point2f> key_points;
        key_points.push_back(cv::Point2f(output.at<float>(i, 0) / scale, output.at<float>(i, 1) / scale));
        key_points.push_back(cv::Point2f(output.at<float>(i, 6) / scale, output.at<float>(i, 7) / scale));
        key_points.push_back(cv::Point2f(output.at<float>(i, 4) / scale, output.at<float>(i, 5) / scale));
        key_points.push_back(cv::Point2f(output.at<float>(i, 2) / scale, output.at<float>(i, 3) / scale));

        // build box
        float min_x = key_points.at(0).x;
        float max_x = key_points.at(0).x;
        float min_y = key_points.at(0).y;
        float max_y = key_points.at(0).y;
        for (size_t j = 1; j < key_points.size(); j++) {
            if (key_points.at(j).x < min_x) min_x = key_points.at(j).x;
            if (key_points.at(j).x > max_x) max_x = key_points.at(j).x;
            if (key_points.at(j).y < min_y) min_y = key_points.at(j).y;
            if (key_points.at(j).y > max_y) max_y = key_points.at(j).y;
        }
        cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

        // add to vector
        confs.emplace_back(conf);
        boxes.emplace_back(rect);
        color_ids.emplace_back(_color_id);
        symbol_ids.emplace_back(_symbol_id);
        all_key_points.emplace_back(key_points);
    }

    // nms
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, this->conf_threshold, this->nms_threshold, indices);

    // construct armor
    for (const auto& ind : indices) {
        Armor armor(color_ids[ind], symbol_ids[ind], confs[ind], all_key_points[ind]);
        
        // fix points
        if (this->fix_points) {
            this->fixPoints(armor, image);
        }

        if (armor.color == this->detect_color) armors.emplace_back(armor);
    }

    return armors;
}

}