#include <armor_detector/tasks/classify.hpp>

namespace pka {

Classifier::Classifier(const std::string& model_path, const std::string& label_path, const double conf, const std::vector<std::string>& ignore_classes) : conf(conf), ignore_classes(ignore_classes) {
    this->net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        this->class_names_.emplace_back(line);
    }
}

void Classifier::classify(std::vector<Armor> &armors) {
    // for each armor
    for (auto& armor : armors) {
        // normalize
        cv::Mat input = armor.pattern / 255.0;

        // create blob from image
        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);

        // set the input blob for the neural network
        this->mutex_.lock();
        this->net_.setInput(blob);

        // forward pass the image blob through the model
        cv::Mat outputs = net_.forward().clone();
        this->mutex_.unlock();

        // output unpack
        // get index & confidence
        double confidence;
        cv::Point index;
        cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &index);

        // get result
        int label_id = index.x;
        armor.conf = confidence;
        armor.symbol_name = this->class_names_[label_id];
        armor.symbol = str2Symbol(armor.symbol_name);
    }

    // erase ignore classes
    this->eraseIgnoreClasses(armors);
}

void Classifier::eraseIgnoreClasses(std::vector<Armor>& armors) {
    armors.erase(
        std::remove_if(
            armors.begin(),
            armors.end(),
            [this](const Armor& armor) {
                if (armor.conf < this->conf) {
                    return true;
                } // confidence judge
                for (const auto& ignore_class : this->ignore_classes) {
                    if (armor.symbol_name == ignore_class) {
                        return true;
                    }
                } // ignore classes judge
                bool mismatch = (((armor.symbol == Symbol::HERO_1 || armor.symbol == Symbol::BASE) && 
                    (armor.type == ArmorType::SMALL || armor.type == ArmorType::INVALID))) || 
                    ((armor.symbol == Symbol::INFANTRY_3 || armor.symbol == Symbol::INFANTRY_4 || armor.symbol == Symbol::INFANTRY_5 || armor.symbol == Symbol::ENGINEER_2 || armor.symbol == Symbol::SENTRY || armor.symbol == Symbol::OUTPOST) && (armor.type == ArmorType::LARGE || armor.type == ArmorType::INVALID));
                if (mismatch) return true; // mismatch judge
                // all ok
                return false;
            }
        ),
        armors.end()
    );
}

}