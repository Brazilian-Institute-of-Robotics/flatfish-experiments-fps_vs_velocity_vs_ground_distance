#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <omp.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frame_helper/FrameHelper.h>

#include <pocolog_cpp/InputDataStream.hpp>
#include <pocolog_cpp/LogFile.hpp>
#include <pocolog_cpp/Index.hpp>


// frame huge transition
//  35391
//  35483
struct ImageFeaturesStruct {

    int idx;
    cv::Mat image;
    std::vector<cv::KeyPoint> key_points;
    cv::Mat descriptors;
};
typedef ImageFeaturesStruct ImageData;

struct InputParamatersStruct {

    std::string directory;
    std::string stream;
    std::string file_name;
    int initial_id;
    int final_id;
    float resize;
    float reduce_boder;

    int clahe_clip;
    int sift_layer;
    int sift_max_features;
    float sift_max_error;

    float fps;
    float max_overlap;
    float min_overlap;

    bool verbose;
};
typedef InputParamatersStruct InputParameters;


// fx: 1993.04
// cx: 995.856
// fy: 1998.89
// cy: 1030.42
// d0: 0.0870984
// d1: 0.0225211
// d2: -0.00145666
// d3: -0.000153369


const cv::Mat camera_k = (cv::Mat_<double>(3,3) <<
                          1993.04, 0, 995.856,
                          0, 1998.89, 1030.42,
                          0, 0, 1);
const cv::Mat camera_dist = (cv::Mat_<double>(1,4) <<
                        0.0870984, 0.0225211, -0.00145666, -0.000153369);

const std::string keys =
  "{d   |     directory|            .|Log file path}"
  "{s   |        stream|             |Log data stream name}"
  "{o   |     file_name|       image_|Prefix image file names}"
  "{i   |    initial_id|            0|Log initial id}"
  "{f   |      final_id|          100|Log final id}"
  "{r   |  resize_image|          0.5|Image resize factor}"
  "{b   |  reduce_boder|          0.1|Delete image boder}"
  "{c   |    clahe_clip|            5|Clahe clip}"
  "{sl  |    sift_layer|           30|Max sift layers}"
  "{sf  | sift_features|        20000|Max sift features}"
  "{se  |sift_max_error|          0.5|Max sift error. Values -> [0.0, 1.0] }"
  "{fps |           FPS|           10|Number of frame per second }"
  "{Mfo |   max_overlap|         0.95|Max overlap percent }"
  "{mfo |   min_overlap|         0.80|Minimum overlap percent }"
  "{v   |       verbose|        false|Verbose procedure }";

void help(cv::CommandLineParser parser){
    parser.printParams();
}

void fillInputParans(cv::CommandLineParser parser,
                     InputParameters &input_params){

    input_params.directory = parser.get<std::string>("d");
    input_params.stream = parser.get<std::string>("s");
    input_params.file_name = parser.get<std::string>("o");
    input_params.initial_id = parser.get<int>("i");
    input_params.final_id = parser.get<int>("f");

    input_params.resize = parser.get<float>("r");
    input_params.reduce_boder = parser.get<float>("b");

    input_params.clahe_clip = parser.get<int>("c");
    input_params.sift_layer = parser.get<int>("sl");
    input_params.sift_max_features = parser.get<int>("sf");
    input_params.sift_max_error = parser.get<float>("se");

    input_params.fps = parser.get<float>("fps");
    input_params.max_overlap = parser.get<float>("Mfo");
    input_params.min_overlap = parser.get<float>("mfo");

    input_params.verbose = parser.get<bool>("v");
}

void printParams(InputParameters input_params){

    std::cout
      << "Input parans :"
      << "\nDirectory         -->  " << input_params.directory
      << "\nStream            -->  " << input_params.stream
      << "\nFile Name         -->  " << input_params.file_name
      << "\nInitial ID        -->  " << input_params.initial_id
      << "\nFinal ID          -->  " << input_params.final_id
      << "\nImage Resize      -->  " << input_params.resize
      << "\nReduce Boder      -->  " << input_params.reduce_boder
      << "\nClahe Clip        -->  " << input_params.clahe_clip
      << "\nSift Layer        -->  " << input_params.sift_layer
      << "\nSift Max Features -->  " << input_params.sift_max_features
      << "\nSift Max Error    -->  " << input_params.sift_max_error
      << "\nFPS               -->  " << input_params.fps
      << "\nMax overlap       -->  " << input_params.max_overlap
      << "\nMin overlap       -->  " << input_params.min_overlap
      << "\nVerbose           -->  " << (input_params.verbose ? "true"
                                                              : "false")
      << std::endl;
}

template <typename T>
std::string num2str ( T Number ){
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

std::string includeZeros(int number, uint digit_size){
    std::string str_num = num2str(number);
    for (uint i = str_num.size() ; i < digit_size; i++) {
        str_num = "0" + str_num;
    }
    return str_num;
}

std::string writeImage(cv::Mat original_image,
                       std::string path,
                       int id,
                       uint digit_size){

    std::string out_file = path + includeZeros(id, digit_size) + ".jpg";
    std::string clahe_file = path +"clahe_" +
                             includeZeros(id, digit_size) + ".jpg";

    //TODO Refactor this code

    cv::Mat image = original_image.clone();

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    std::vector<cv::Mat> channels;
    cv::cvtColor(image, image, CV_BGR2HSV);
    cv::split(image, channels);

    clahe->setClipLimit( 4 );
    clahe->apply( channels[2], channels[2] );

    cv::merge(channels, image);
    cv::cvtColor(image, image, CV_HSV2BGR);
    cv::imwrite(clahe_file, image);

    cv::imwrite(out_file, original_image);
    return out_file;
}


void reduceBoder(cv::Mat& image, float percent){
    cv::Point top_letf( image.cols * percent, image.rows * percent );
    float factor = (1.0 - 2*percent);
    cv::Size size( image.rows*factor, image.cols*factor );
    image = image( cv::Rect(top_letf, size) );
}

void applyClahe(cv::Mat& image, cv::Ptr<cv::CLAHE> clahe, int clip_limit ){

    if(clip_limit > 0){
        std::vector<cv::Mat> channels;
        cv::cvtColor(image, image, CV_BGR2HSV);
        cv::split(image, channels);

        clahe->setClipLimit( clip_limit );
        clahe->apply( channels[2], channels[2] );

        cv::merge(channels, image);
        cv::cvtColor(image, image, CV_HSV2BGR);
    }
}

void computeSift(
        ImageData& img_features,
        cv::Ptr<cv::FeatureDetector> feature_detector,
        cv::Ptr<cv::DescriptorExtractor> descriptor_extractor,
        cv::Ptr<cv::CLAHE> clahe,
        int clip_limit = 5 ){

    cv::Mat image = img_features.image.clone();
    applyClahe(image, clahe, clip_limit);

    feature_detector->detect(image, img_features.key_points);
    descriptor_extractor->compute( image,
                                   img_features.key_points,
                                   img_features.descriptors);
}

cv::Mat outResize(cv::Mat& image, int size_limit = 1000 ){

    float reference = 0;
    if(image.cols > image.rows)
        reference = image.cols;
    else
        reference = image.rows;

    reference = size_limit * 1.0/reference;
    cv::Mat out_image;
    cv::resize(image, out_image, cv::Size(), reference, reference);

    return out_image;
}

void computeHomografy(
            std::vector<cv::DMatch>& matchers,
            std::vector<cv::KeyPoint>keypoints_1,
            std::vector<cv::KeyPoint>keypoints_2,
            cv::Mat &homografy,
            int reproj_threshold = 3 ){

    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (uint i = 0; i < matchers.size(); i++) {
        obj.push_back(keypoints_1[matchers[i].queryIdx].pt);
        scene.push_back(keypoints_2[matchers[i].trainIdx].pt);
    }

    cv::Mat selectedSet;
    homografy = cv::findHomography( obj, scene, CV_RANSAC,
                                    reproj_threshold, selectedSet);

    std::vector<cv::DMatch> final_matchers;
    for (int i = 0; i < selectedSet.rows; ++i) {
        if (selectedSet.at<int>(i))
            final_matchers.push_back(matchers[i]);
    }
    matchers = final_matchers;
}

std::vector<cv::DMatch> filterMatchersByDistance(
                                          std::vector<cv::DMatch> matchers,
                                          double percernt_Max = 0.5) {
    double max_dist = 0;

    //-- Quick calculation of max and min distances between keypoints
    for (uint i = 0; i < matchers.size(); ++i) {
        double dist = matchers[i].distance;
        if (dist > max_dist)
            max_dist = dist;
    }

    std::vector<cv::DMatch> good_matches;
    for (uint i = 0; i < matchers.size(); ++i) {
        if (matchers[i].distance <= (max_dist * percernt_Max)) {
            good_matches.push_back(matchers[i]);
        }
    }

  return good_matches;
}

int intersectArea( std::vector<cv::Point2f> points_1,
                     std::vector<cv::Point2f> points_2){
    int area = 0;
    cv::Rect min_rect = cv::boundingRect(points_1);
    for (int y = min_rect.tl().y; y < min_rect.size().width; y++)
        for (int x = min_rect.tl().x; x < min_rect.size().height; x++)
            if( pointPolygonTest( points_1, cv::Point2f(x,y), false ) > 0
               && pointPolygonTest( points_2, cv::Point2f(x,y), false ) > 0 ){
                 area++;
               }

    return area;
 }

int main(int argc, char const *argv[]) {

  cv::CommandLineParser parser(argc, argv, keys.c_str());

  if(argc < 3){
    help(parser);
    return 0;
  }

  InputParameters input_parameters;
  fillInputParans(parser, input_parameters);
  printParams(input_parameters);

  int diff = input_parameters.final_id - input_parameters.initial_id;
  if(diff < 1){
    std::cout << "ERROR => initial_id must be greater than final_id."
              << std::endl;
    return 0;
  }

  uint digit_size = num2str(diff).size();

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  cv::Ptr<cv::FeatureDetector> feature_detector;
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;

  feature_detector = new cv::SiftFeatureDetector(
                                          input_parameters.sift_max_features,
                                          input_parameters.sift_layer);

  descriptor_extractor = new cv::SiftDescriptorExtractor(
                                          input_parameters.sift_max_features,
                                          input_parameters.sift_layer);

  descriptor_matcher = new cv::BFMatcher(cv::NORM_L2, true);

  cv::Mat map1, map2;
  cv::Size im_size(2040,2040);
  cv::initUndistortRectifyMap(camera_k, camera_dist, cv::Mat(), camera_k,
                              im_size, CV_16SC2, map1, map2);

  std::cout << "Loading stream ..." << std::endl;
  pocolog_cpp::InputDataStream* stream;
  pocolog_cpp::LogFile log_file(input_parameters.directory);
  stream = dynamic_cast<pocolog_cpp::InputDataStream*>(
                                &(log_file.getStream(input_parameters.stream)));

  base::samples::frame::Frame frame_rock;
  ImageData current_image, prev_image;
  uint j = 0;
  for (int i = input_parameters.initial_id;
       i < input_parameters.final_id
          && stream->getSample<base::samples::frame::Frame>(frame_rock, i);
       i++) {

      std::cout << "\nRead frame #"<< i << std::endl;
      ImageData temp_image;
      cv::Mat image_cv = frame_helper::FrameHelper::convertToCvMat(frame_rock);
      cv::cvtColor(image_cv, image_cv, CV_BayerGR2RGB);
      cv::remap(image_cv, image_cv, map1, map2, cv::INTER_CUBIC);

      reduceBoder(image_cv, input_parameters.reduce_boder);
      cv::resize( image_cv,
                  image_cv,
                  cv::Size(),
                  input_parameters.resize,
                  input_parameters.resize );

      temp_image.idx = i;
      temp_image.image = image_cv;
      computeSift(temp_image,
                  feature_detector,
                  descriptor_extractor,
                  clahe,
                  input_parameters.clahe_clip);

      std::cout << " Key Points #" << temp_image.key_points.size()
                << " Descriptor " << temp_image.descriptors.size()
                << std::endl;

      if( current_image.image.empty() ){
          current_image = temp_image;
          std::string out_file = writeImage(current_image.image,
                                            input_parameters.file_name,
                                            j++,
                                            digit_size);
          std::cout << " Write file: "<< out_file << std::endl;
          continue;
      }

      std::vector<cv::DMatch> matchs;
      descriptor_matcher->match( temp_image.descriptors,
                                 current_image.descriptors,
                                 matchs);

      matchs = filterMatchersByDistance(matchs,
                                        input_parameters.sift_max_error);

      if(matchs.size() < 10){
        std::cout << " Ignore image: low number of matchs < 10 " << std::endl;
        continue;
      }

      
      cv::Mat homografy;
      computeHomografy(matchs,
                       temp_image.key_points,
                       current_image.key_points,
                       homografy);


      if(input_parameters.verbose){
          std::cout << " Number Matchs # " << matchs.size() << std::endl;
          cv::Mat image_out;
          cv::drawMatches(temp_image.image,
                          temp_image.key_points,
                          current_image.image,
                          current_image.key_points,
                          matchs, image_out, cv::Scalar(0,0,255));
          image_out = outResize(image_out, 1000);
          cv::imshow("Matches", image_out);
      }

      std::vector<cv::Point2f> out_point, in_points, local_point(4);
      local_point[0] = cv::Point(0,0);
      local_point[1] = cv::Point(current_image.image.cols, 0);
      local_point[2] = cv::Point(current_image.image.cols,
                                 current_image.image.rows);
      local_point[3] = cv::Point(0, current_image.image.rows);

      cv::Mat temp_homografy = homografy.clone();
      cv::perspectiveTransform(local_point, out_point, temp_homografy);
      cv::perspectiveTransform(local_point, in_points, temp_homografy.inv());

      if(input_parameters.verbose){
          cv::Mat draw_image = current_image.image.clone();
          line(draw_image, out_point[0], out_point[1], cv::Scalar(0,0,255),4);
          line(draw_image, out_point[1], out_point[2], cv::Scalar(0,0,255),4);
          line(draw_image, out_point[2], out_point[3], cv::Scalar(0,0,255),4);
          line(draw_image, out_point[3], out_point[0], cv::Scalar(0,0,255),4);
          cv::imshow( "MAIN IMAGE OVERLAP  ", draw_image);
      }

      float overlap = intersectArea(local_point, out_point);
      overlap = overlap/current_image.image.size().area();
      std::cout << " Image area overlap -> "<< overlap << std::endl;


      if(input_parameters.verbose)
          cv::waitKey(30);

      if( input_parameters.max_overlap < overlap ){
          prev_image = temp_image;
          continue;
      }

      if( overlap < input_parameters.min_overlap){
          std::string out_file = writeImage(prev_image.image,
                                            input_parameters.file_name,
                                            j++,
                                            digit_size);
          std::cout << " Write previus file: "<< out_file << std::endl;
      }

      current_image = temp_image;
      std::string out_file = writeImage(current_image.image,
                                        input_parameters.file_name,
                                        j++,
                                        digit_size);
      std::cout << " Write file: "<< out_file << std::endl;
  }

  delete stream;
  return 0;
}
