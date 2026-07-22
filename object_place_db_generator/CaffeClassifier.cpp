#include "vision/CaffeClassifier.h"

CaffeClassifier::CaffeClassifier(const std::string& model_file, const std::string& trained_file,
                                 const std::string& mean_file, const std::string& label_file){
  caffe::Caffe::set_mode(caffe::Caffe::Brew(1));

  /* Load the network. */
  net_ = new caffe::Net<float>(model_file, caffe::TEST);
  net_->CopyTrainedLayersFrom(trained_file);

  if(net_->num_inputs() != 1){
    std::cout << "Network should have exactly one input.";
    return;
  }
  if(net_->num_outputs() != 1){
    std::cout << "Network should have exactly one output.";
    return;
  }

  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  if(num_channels_ != 3 && num_channels_ != 1){
    std::cout << "Input layer should have 1 or 3 channels.";
    return;
  }
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
  std::cout << "INPUT GEOMETRY (w,h): " << input_geometry_.width << " " << input_geometry_.height << std::endl;

  /* Load the binaryproto mean file. */
  setMean(mean_file);

  /* Load labels. */
  std::ifstream labels(label_file.c_str());
  if(!labels.good()){
    std::cout << "Unable to open labels file " << label_file;
    return;
  }
  std::string line;
  while (std::getline(labels, line))
    labels_.push_back(std::string(line));
  for(int i=0; i<labels_.size(); i++){
    used_classes_.push_back((labels_[i][0] != '_'));
  }

  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
  if (labels_.size() != output_layer->channels()){
    std::cout << "Number of labels is different from the output layer dimension.";
    return;
  }
  is_ok_ = true;
}

CaffeClassifier::~CaffeClassifier(){
  delete net_;
}

static bool pairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> argmax(const std::vector<float>& v, int N) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), pairCompare);

  std::vector<int> result;
  for (int i = 0; i < N; ++i)
    result.push_back(pairs[i].second);
  return result;
}

/* Return all without ordering */
std::vector<CaffeRecognition> CaffeClassifier::classify(const cv::Mat& img){
  std::vector<float> output = predict(img);
  std::vector<CaffeRecognition> predictions;
  double sum = 0.0;
  for(int i = 0; i < output.size(); ++i){
    if(used_classes_[i]){
      predictions.push_back(CaffeRecognition(labels_[i], i, output[i]));
      sum += output[i];
    }
  }
  for(auto& pred : predictions){
    pred.prob_ /= sum;
  }

  return predictions;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> argmax(const std::vector<float>& v, float min_prob) {
  std::vector<std::pair<float, int> > pairs;
  for (size_t i = 0; i < v.size(); ++i)
    pairs.push_back(std::make_pair(v[i], i));
  std::sort(pairs.begin(), pairs.end(), pairCompare);

  std::vector<int> result;
  for (int i = 0; i < pairs.size() && pairs[i].first >= min_prob; ++i)
    result.push_back(pairs[i].second);
  return result;
}

/* Return the top N std::pair<float, int>s. */
std::vector<CaffeRecognition> CaffeClassifier::classify(const cv::Mat& img, int N) {
  std::vector<float> output = predict(img);

  N = std::min<int>(labels_.size(), N);
  std::vector<int> maxN = argmax(output, N);
  std::vector<CaffeRecognition> predictions;
  for (int i = 0; i < N; ++i) {
    int idx = maxN[i];
    predictions.push_back(CaffeRecognition(labels_[idx], idx, output[idx]));
  }

  return predictions;
}

std::vector<CaffeRecognition> CaffeClassifier::classify(const cv::Mat& img, float min_prob){
  std::vector<float> output = predict(img);

  std::vector<int> maxN = argmax(output, min_prob);
  std::vector<CaffeRecognition> predictions;
  for (int i = 0; i < maxN.size(); ++i) {
    int idx = maxN[i];
    predictions.push_back(CaffeRecognition(labels_[idx], idx, output[idx]));
  }

  return predictions;
};

/* Load the mean file in binaryproto format. */
void CaffeClassifier::setMean(const std::string& mean_file) {
  if(mean_file.empty())
    mean_ = cv::Mat(input_geometry_, CV_32FC3, cv::Scalar(104.0065, 116.6690, 122.6795));
  else{
    mean_ = cv::imread(mean_file, cv::IMREAD_UNCHANGED);
  }
}

std::vector<float> CaffeClassifier::predict(const cv::Mat& img) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  wrapInputLayer(&input_channels);

  preprocess(img, &input_channels);

  net_->Forward();

  /* Copy the output layer to a std::vector */
  caffe::Blob<float>* output_layer = net_->output_blobs()[0];
  const float* begin = output_layer->cpu_data();
  const float* end = begin + output_layer->channels();
  return std::vector<float>(begin, end);
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void CaffeClassifier::wrapInputLayer(std::vector<cv::Mat>* input_channels) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void CaffeClassifier::preprocess(const cv::Mat& img,
                                 std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  std::cout << "TYPES " << sample_float.type() << " " << mean_.type() << " " << std::endl;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  if(reinterpret_cast<float*>(input_channels->at(0).data) != net_->input_blobs()[0]->cpu_data())
    std::cout << "Input channels are not wrapping the input layer of the network.";
}