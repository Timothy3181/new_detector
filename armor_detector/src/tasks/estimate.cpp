#include <armor_detector/tasks/estimate.hpp>

namespace pka {

Estimator::Estimator(const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info, const bool& optimized_yaw, const bool solve_in_camera, const double& search_range) : optimize_yaw_switch(optimized_yaw), solve_in_camera(solve_in_camera), search_range(search_range) {
    this->camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info->k.data())).clone();
    this->distort_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(camera_info->d.data())).clone();
}

void Estimator::setTFRelationship(Eigen::Matrix3d& R_g2o, Eigen::Vector3d& t_g2o, Eigen::Matrix3d& R_c2g, Eigen::Vector3d& t_c2g) {
    this->R_gimbal2odom_ = R_g2o;
    this->t_gimbal2odom_ = t_g2o;
    this->R_camera2gimbal_ = R_c2g;
    this->t_camera2gimbal_ = t_c2g;
}

std::vector<rm_interfaces::msg::Armor> Estimator::estimate(std::vector<Armor>& armors) {
    // init msg Armors
    std::vector<rm_interfaces::msg::Armor> armors_msg;

    // estimate main func
    for (auto& armor : armors) {
        // select 3-d armor points
        const auto object_points = 
            (armor.type == ArmorType::LARGE) ? LARGE_ARMOR_POINTS : SMALL_ARMOR_POINTS;

        // solve PnP
        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points, armor.points, this->camera_matrix_, this->distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

        // turn rvec to rotation matrix
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(rmat, R_armor2camera);

        // deal with tvec
        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);

        // cal armor rpy
        armor.rpy_in_camera = rotationMatrix2RPY(R_armor2camera);
        auto R_armor2gimbal = R_armor2camera * this->R_camera2gimbal_;
        armor.rpy_in_gimbal = rotationMatrix2RPY(R_armor2gimbal);
        auto R_armor2odom = R_armor2gimbal * this->R_gimbal2odom_;
        armor.rpy_in_odom = rotationMatrix2RPY(R_armor2odom);

        // cal armor xyz
        armor.xyz_in_camera = xyz_in_camera;
        armor.xyz_in_gimbal = this->R_camera2gimbal_ * xyz_in_camera + this->t_camera2gimbal_;
        armor.xyz_in_odom = this->R_gimbal2odom_ * armor.xyz_in_gimbal + this->t_gimbal2odom_;

        // optimize yaw
        bool is_balance = (armor.type == ArmorType::LARGE) && 
            (armor.symbol == Symbol::INFANTRY_3 || armor.symbol == Symbol::INFANTRY_4 || armor.symbol == Symbol::INFANTRY_5);
        if (!is_balance && this->optimize_yaw_switch) {
            this->optimizeYaw(armor);
        }

        // turn rpy to quaternion
        auto q = rpy2Quaternion(armor.rpy_in_odom);
        auto t = armor.xyz_in_odom;

        // init armor msg
        rm_interfaces::msg::Armor armor_msg;

        // send in camera or odom
        if (!this->solve_in_camera) {
            // in odom
            // type
            armor_msg.type = armor.type_name;
            // symbol
            armor_msg.number = armor.symbol_name;
            // pose
            armor_msg.pose.position.x = t[0];
            armor_msg.pose.position.y = t[1];
            armor_msg.pose.position.z = t[2];
            armor_msg.pose.orientation.w = q.w();
            armor_msg.pose.orientation.x = q.x();
            armor_msg.pose.orientation.y = q.y();
            armor_msg.pose.orientation.z = q.z();
            // to center dist
            armor_msg.distance_to_image_center = cal2CenterDist(armor.center);
        } else {
            // in camera
            Eigen::Matrix3d R_armor2odom = q.toRotationMatrix();
            Eigen::Matrix3d R_armor2camera = this->R_camera2gimbal_.transpose() * this->R_gimbal2odom_.transpose() * R_armor2odom;
            Eigen::Quaterniond q_camera(R_armor2camera);
            auto t_camera = armor.xyz_in_camera;
            // type 
            armor_msg.type = armor.type_name;
            // symbol
            armor_msg.number = armor.symbol_name;
            // pose
            armor_msg.pose.position.x = t_camera[0];
            armor_msg.pose.position.y = t_camera[1];
            armor_msg.pose.position.z = t_camera[2];
            armor_msg.pose.orientation.w = q_camera.w();
            armor_msg.pose.orientation.x = q_camera.x();
            armor_msg.pose.orientation.y = q_camera.y();
            armor_msg.pose.orientation.z = q_camera.z();
            // to center dist
            armor_msg.distance_to_image_center = cal2CenterDist(armor.center);
        }

        // push back
        armors_msg.push_back(std::move(armor_msg));
    }
    return armors_msg;
}

float Estimator::cal2CenterDist(const cv::Point2f& image_point) {
    auto cam_center_x = this->camera_matrix_.at<double>(0, 2);
    auto cam_center_y = this->camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cam_center_x, cam_center_y));
}

double Estimator::armorReprojectionError(const Armor& armor, double yaw) {
    // object points
    auto object_points = (armor.type == ArmorType::LARGE) ? LARGE_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    
    // output 2-d points
    std::vector<cv::Point2f> two_dimension_points;

    // rvec
    auto cos_yaw = std::cos(yaw);
    auto sin_yaw = std::sin(yaw);
    double pitch = (armor.symbol == Symbol::OUTPOST) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
    auto cos_pitch = std::cos(pitch);
    auto sin_pitch = std::sin(pitch);

    const Eigen::Matrix3d R_armor2odom_fix {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
        {         -sin_pitch,        0,           cos_pitch}
    };

    Eigen::Matrix3d R_armor2camera_fix = this->R_camera2gimbal_.transpose() * this->R_gimbal2odom_.transpose() * R_armor2odom_fix;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera_fix, R_armor2camera_cv);
    cv::Vec3d rvec;
    cv::Rodrigues(R_armor2camera_cv, rvec);

    // tvec
    const Eigen::Vector3d t_armor2camera = armor.xyz_in_camera;
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // inverse pnp
    cv::projectPoints(object_points, rvec, tvec, this->camera_matrix_, this->distort_coeffs_, two_dimension_points);

    // cal error
    double error = 0;
    for (int i = 0; i < 4; i++) {
        error += cv::norm(two_dimension_points[i] - armor.points[i]);
    }

    // result
    return error;
}

void Estimator::optimizeYaw(Armor& armor) {
    // init
    Eigen::Vector3d gimbal_rpy = rotationMatrix2RPY(this->R_gimbal2odom_);
    auto yaw0 = limit_rad(gimbal_rpy[2] - (this->search_range / 2) * CV_PI / 180.0);
    double min_error = 1e10;
    auto best_yaw = armor.rpy_in_odom[2];

    // 迭代法寻找最优yaw
    for (double i = 0; i < this->search_range; i++) {
        auto yaw = limit_rad(yaw0 + i * CV_PI / 180.0);
        auto error = this->armorReprojectionError(armor, yaw);
        if (error < min_error) {
            best_yaw = yaw;
            min_error = error;
        }
    }

    // update yaw
    armor.yaw_raw = armor.rpy_in_odom[2];
    armor.rpy_in_odom[2] = best_yaw;
}

}