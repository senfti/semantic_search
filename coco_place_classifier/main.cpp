#include "CaffeClassifier.h"

const std::string model_file   = "/home/thomas/BVLCcaffe/models/Places205-VGGNet-master/siat_scene_vgg_16_deploy.prototxt";
const std::string trained_file = "/home/thomas/BVLCcaffe/models/Places205-VGGNet-master/siat_scene_vgg_16.caffemodel";
const std::string mean_file    = "/home/thomas/BVLCcaffe/models/Places205-VGGNet-master/places205_mean.exr";
const std::string label_file   = "/home/thomas/BVLCcaffe/models/placescnn/categories_places205.csv";
const float min_place_prob = 0.f;

std::string input_file = "/home/thomas/coco_data_objects.txt";

int main(){
  CaffeClassifier classifier = CaffeClassifier(model_file, trained_file, mean_file, label_file);

  std::ifstream file(input_file);
  std::vector<std::string> image_files;
  std::string part;
  while(true){
    file >> part;
    if(file.eof())
      break;
    if(part[0] == '/'){
      image_files.push_back(part);
    }
  }
  std::cout << "image_files " << image_files.size() << std::endl;

  std::ofstream fout("/home/thomas/coco_places.txt");
  for(int i=0; i<image_files.size(); i++){
    cv::Mat img = cv::imread(image_files[i]);
    std::vector<CaffeRecognition> predictions = classifier.classify(img, min_place_prob);

    std::vector<float> probs(205);
    for(int j=0; j<predictions.size(); j++){
      probs[predictions[j].id_] = predictions[j].prob_;
    }

    for(int j=0; j<probs.size(); j++){
      fout << probs[j] << " ";
    }
    fout << std::endl;

    std::cout << i << "    " << image_files[i] << std::endl;
  }

  return 0;
}