/*
 * ImageAnalysis_test.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: tiagotrocoli
 */
#define BOOST_TEST_MODULE "ImageAnalysis_test"
#include <boost/test/unit_test.hpp>

#include <iostream>
#include <limits>

#include <pocolog_cpp/InputDataStream.hpp>
#include <base/samples/Frame.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <frame_helper/FrameHelper.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TestTools.hpp"
#include <fps_per_velocity/PocologTools.hpp>
#include <fps_per_velocity/ImageAnalysisTool.hpp>
#include <fps_per_velocity/CLAHE.hpp>
#include <fps_per_velocity/ImageFeartureDetectorTools.hpp>

using namespace fps_per_velocity;

std::string temp_path_camera = std::string(PATH_LOG_PAYLOAD) + "/"
    + std::string(LOG_CAMERA_DOWN_LEFT);
std::string temp_stream_camera = std::string(STREAM_CAMERA_DOWN_LEFT);

std::string temp_path_dvl = std::string(PATH_LOG_NAV) + "/"
    + std::string(LOG_DVL);
std::string temp_stream_dvl_velocity = std::string(STREAM_DVL_VELOCITY);
std::string temp_stream_dvl_ground_distance = std::string(
STREAM_DVL_GROUND_DISTANCE);

pocolog_cpp::InputDataStream* video_stream = PocologTools::getDataStreamFromLog(
    temp_path_camera, temp_stream_camera);
pocolog_cpp::InputDataStream* dvl_stream = PocologTools::getDataStreamFromLog(
    temp_path_dvl, temp_stream_dvl_velocity);
pocolog_cpp::InputDataStream* dvl_ground_stream =
    PocologTools::getDataStreamFromLog(temp_path_dvl,
                                       temp_stream_dvl_ground_distance);

ImageDetectorDescriptor detectorAndDescriptor;
ImageAnalysisTool image_analysis;

cv::Size gt_size(189, 274);
// for  data_stream_size_percent 0.05
cv::Mat gt_overlap_areas =
    (cv::Mat_<double>(1, 128) << 0.9909650134563629, 0.969281045751634, 0.977677816224529, 0.9566897347174164, 0.9223644752018454, 0.9336101499423298, 0.8777893118031527, 0.7783698577470204, 0.7865090349865437, 0.7770780469050366, 0.8225451749327182, 0.8570934256055364, 0.8863821607074203, 0.9177008842752787, 0.9340868896578239, 0.9338985005767013, 0.9372164552095348, 0.9297500961168781, 0.9620184544405997, 0.9375278738946559, 0.9560745866974241, 0.9554171472510573, 0.9842445213379469, 0.9858016147635524, 0.9702960399846213, 0.9634525182622068, 0.9764782775855441, 0.974479046520569, 0.9681276432141485, 0.9849750096116878, 0.9687620146097655, 0.9642599000384467, 0.9862898885044213, 0.9763283352556709, 0.9904421376393695, 0.959923106497501, 0.9657785467128027, 0.9775816993464053, 0.9803267973856209, 0.9761707035755479, 0.9762668204536716, 0.9723068050749711, 0.9870972702806613, 0.9819530949634756, 0.9458592848904268, 0.905121107266436, 0.9286236063052672, 0.8802806612841215, 0.8920146097654749, 0.9832410611303345, 0.9480738177623991, 0.9594271434063821, 0.9803575547866206, 0.9567320261437908, 0.9373125720876586, 0.9536985774702038, 0.9397731641676278, 0.9505420991926182, 0.974640522875817, 0.9781276432141484, 0.9239869281045752, 0.9563783160322953, 0.9426066897347174, 0.9496655132641292, 0.9345174932718185, 0.9414455978469819, 0.9317147251057286, 0.9454094579008074, 0.9517916186082276, 0.9603883121876201, 0.9757247212610535, 0.9788888888888889, 0.9964859669357939, 0.9837254901960785, 0.9824183006535948, 0.9787389465590157, 0.9676278354479046, 0.9685736255286428, 0.9735832372164552, 0.9711072664359861, 0.9736601307189543, 0.9876470588235294, 0.9752210688196847, 0.9533833141099577, 0.9689311803152634, 0.9997731641676278, 0.9888235294117647, 0.9865513264129181, 0.9662129950019224, 0.9820991926182238, 0.9229334871203383, 0.9153902345251826, 0.9183044982698962, 0.9312918108419839, 0.9346597462514418, 0.9232295271049596, 0.8988196847366398, 0.8553018069973087, 0.9087697039600153, 0.9569434832756633, 0.9791503267973857, 0.9660092272202999, 0.9860092272202999, 0.9796347558631295, 0.9488619761630143, 0.9591541714725106, 0.9559746251441753, 0.9792733564013841, 0.9380123029603998, 0.9698000768935024, 0.972483660130719, 0.9730834294502114, 0.9653479430988081, 0.9698654363706267, 0.9776624375240293, 0.956724336793541, 0.9650480584390619, 0.9754325259515571, 0.9864552095347943, 0.9736332179930796, 0.9729719338715879, 0.9729719338715879, 0.9849058054594387, 0.9947558631295655, 0.9780430603613994, 0.9724605920799693, 0.9543560169165706, 0.9913648596693579);

cv::Mat gt_relation =
    (cv::Mat_<double>(128, 2) << 0.0002610000182744119, 0.9909650134563629, 0.000617000070951881, 0.969281045751634, 0.0008080000227391722, 0.977677816224529, 0.00174200009609759, 0.9566897347174164, 0.008237001123569955, 0.9223644752018454, 0.005621000341415411, 0.9336101499423298, 0.02173900169885162, 0.8777893118031527, 0.04486100199589137, 0.7783698577470204, 0.04956200335818535, 0.7865090349865437, 0.06986600263641778, 0.7770780469050366, 0.03719400321146853, 0.8225451749327182, 0.04208400296306615, 0.8570934256055364, 0.01706900264056782, 0.8863821607074203, 0.01436500146076087, 0.9177008842752787, 0.003781000209465626, 0.9340868896578239, 0.006961000713646432, 0.9338985005767013, 0.0002500000214204197, 0.9372164552095348, 0.002835000217622151, 0.9297500961168781, 0.001254000115863982, 0.9620184544405997, 0.001650000041723252, 0.9375278738946559, 0.001085000084675851, 0.9560745866974241, 0.0003940000192672018, 0.9554171472510573, 0.001104000078797342, 0.9842445213379469, 0.0005210000487938535, 0.9858016147635524, 0.001251000113949183, 0.9702960399846213, 0.0004820000713989165, 0.9634525182622068, 0.001874000219464309, 0.9764782775855441, 0.00202100004041195, 0.974479046520569, 0.001349000006377697, 0.9681276432141485, 0.0009450000723078861, 0.9849750096116878, 0.001949000172339384, 0.9687620146097655, 0.003107000120759013, 0.9642599000384467, 0.0008360000347122556, 0.9862898885044213, 0.0004140000491067782, 0.9763283352556709, 0.005336000505030167, 0.9904421376393695, 0.005146000479996221, 0.959923106497501, 0.0007470000697970408, 0.9657785467128027, 0.0002090000079795719, 0.9775816993464053, 0.0001010000116899613, 0.9803267973856209, 0.0001340000066757203, 0.9761707035755479, 0.001299000118508938, 0.9762668204536716, 0.0007100000902637868, 0.9723068050749711, 0.001233000109910967, 0.9870972702806613, 0.002558000077687204, 0.9819530949634756, 0.0147680011123121, 0.9458592848904268, 0.027585002616942, 0.905121107266436, 0.02398800159049037, 0.9286236063052672, 0.02469800207656626, 0.8802806612841215, 0.01130000042617322, 0.8920146097654749, 0.00624200035873056, 0.9832410611303345, 0.002395000008419156, 0.9480738177623991, 0.00233300035503508, 0.9594271434063821, 0.004326000403031716, 0.9803575547866206, 0.00451700037996472, 0.9567320261437908, 0.0009020000451728707, 0.9373125720876586, 0.0006050000642240068, 0.9536985774702038, 0.0001260000177901244, 0.9397731641676278, 5.200000214576725e-05, 0.9505420991926182, 0.001552000076651575, 0.974640522875817, 0.003249000456020252, 0.9781276432141484, 0.01134900105690959, 0.9239869281045752, 0.02575400173357132, 0.9563783160322953, 0.02545400382196918, 0.9426066897347174, 0.02069300015547127, 0.9496655132641292, 0.01363700157228117, 0.9345174932718185, 0.009189001434728561, 0.9414455978469819, 0.008708000913828638, 0.9317147251057286, 0.009221001415416652, 0.9454094579008074, 0.00735000110566621, 0.9517916186082276, 0.004089000271320348, 0.9603883121876201, std::numeric_limits<
        double>::quiet_NaN(), 0.9757247212610535, 0.004509000483512896, 0.9788888888888889, 0.001154000069111587, 0.9964859669357939, 0.000941000029087067, 0.9837254901960785, std::numeric_limits<
        double>::quiet_NaN(), 0.9824183006535948, 0.0001950000257417569, 0.9787389465590157, 0.002801000437676924, 0.9676278354479046, 0.001409000036291778, 0.9685736255286428, 0.0007050000318139795, 0.9735832372164552, 0.003734000262044375, 0.9711072664359861, 0.001754000202476985, 0.9736601307189543, std::numeric_limits<
        double>::quiet_NaN(), 0.9876470588235294, 0.001553000049971045, 0.9752210688196847, 0.006029000527322304, 0.9533833141099577, 0.002788000362634671, 0.9689311803152634, 0.003170000055730343, 0.9997731641676278, 0.003891000266015535, 0.9888235294117647, 7.500001061707773e-05, 0.9865513264129181, std::numeric_limits<
        double>::quiet_NaN(), 0.9662129950019224, std::numeric_limits<double>::quiet_NaN(), 0.9820991926182238, std::numeric_limits<
        double>::quiet_NaN(), 0.9229334871203383, 0.03625300475911066, 0.9153902345251826, 0.07759500682234788, 0.9183044982698962, 0.04776900731897382, 0.9312918108419839, 0.02855200428529845, 0.9346597462514418, 0.009189000400960452, 0.9232295271049596, 0.02133800071430209, 0.8988196847366398, 0.02594600054481626, 0.8553018069973087, 0.01555500226065525, 0.9087697039600153, 0.006181000197172173, 0.9569434832756633, 0.00760100023055077, 0.9791503267973857, 0.009502000682383788, 0.9660092272202999, 0.006629001088164792, 0.9860092272202999, 0.004121000266909605, 0.9796347558631295, 0.006281000706791898, 0.9488619761630143, 0.01034600033508241, 0.9591541714725106, 0.001997000188075011, 0.9559746251441753, 0.001699000085726384, 0.9792733564013841, 0.0008570000546351084, 0.9380123029603998, 0.001291000102847818, 0.9698000768935024, 0.01555300144416097, 0.972483660130719, 0.004525000213086609, 0.9730834294502114, 0.00138000011712313, 0.9653479430988081, 0.0009630000530630361, 0.9698654363706267, 0.00446900037703478, 0.9776624375240293, 0.004748000304818159, 0.956724336793541, 0.00226600010303408, 0.9650480584390619, 0.002228000305712234, 0.9754325259515571, 0.003090000356398533, 0.9864552095347943, 0.003194000179082158, 0.9736332179930796, 0.0005960000119134784, 0.9729719338715879, 0.001013000066660346, 0.9729719338715879, 0.00314100044832194, 0.9849058054594387, 0.001358000109910967, 0.9947558631295655, 0.0001090000143125658, 0.9780430603613994, 0.00248300014693104, 0.9724605920799693, 0.003373000492014009, 0.9543560169165706, 0.002563000248827047, 0.9913648596693579);

float percent_image_size = 0.25;

template<typename Type>
std::string toStr(const Type& temp) {
  std::ostringstream os;
  os << temp;
  return os.str();
}

BOOST_AUTO_TEST_CASE(AreaOverlapCalc_testCase) {

  std::string path_image1 = std::string(PATH_RESOURCE) + "/"
      + std::string(IMAGE_01);
  std::string path_image2 = std::string(PATH_RESOURCE) + "/"
      + std::string(IMAGE_02);

  cv::Mat target_image = cv::imread(path_image1);
  cv::Mat query_image = cv::imread(path_image2);

  ImageFeaturesDescriptor target_image_features = detectorAndDescriptor
      .applyFeatureDetectorAndDescriptorExtractor(target_image);
  ImageFeaturesDescriptor query_image_features = detectorAndDescriptor
      .applyFeatureDetectorAndDescriptorExtractor(query_image);

  std::vector<cv::DMatch> matches =
      detectorAndDescriptor.applyDescriptorMatcher(
          target_image_features.descriptor(),
          query_image_features.descriptor());

  matches = detectorAndDescriptor.filterMatchersByDistance(matches, 0.2);

  cv::Mat homografy;
  matches = detectorAndDescriptor.filterMatchersByRansacHomografy(
      matches, target_image_features.key_points(),
      query_image_features.key_points(), 0, &homografy);

  cv::Mat out;
  double area_overlaped = image_analysis.calcOverlapPercent(
      target_image_features.image(), query_image_features.image(), homografy,
      &out);

  BOOST_CHECK_CLOSE(area_overlaped,
                    gt_size.area() / (1.0 * query_image.size().area()), 1);

}

BOOST_AUTO_TEST_CASE(CLAHE_testcase) {

  cv::Mat image_clahe, image_original = PocologTools::getMatFromStream<
      base::samples::frame::Frame>(video_stream, 100, percent_image_size);

  image_original.copyTo(image_clahe);

  SingleImageHazeRemoval *hazeRemoval = new CLAHE();
  ImageFeaturesDescriptor image_original_features, image_clahe_features;

//  image_original_features = detectorAndDescriptor
//      .applyFeatureDetectorAndDescriptorExtractor(image_original);

  hazeRemoval->applyHazeRemoval(image_clahe);
  image_clahe_features = detectorAndDescriptor
      .applyFeatureDetectorAndDescriptorExtractor(image_clahe);

  BOOST_CHECK_EQUAL(1500, image_clahe_features.key_points().size());

// show keypoints
//
//  cv::imshow("original ", image_original);
//  cv::imshow("CLAHE ", image_clahe);
//
//  cv::Mat image_out_original;
//  cv::Mat image_out_clahe;
//
//  cv::drawKeypoints(image_original, image_original_features.key_points(),
//                    image_out_original);
//  cv::drawKeypoints(image_clahe, image_clahe_features.key_points(),
//                    image_out_clahe);
//
//  cv::imshow("original key", image_out_original);
//  cv::imshow("CLAHE key", image_out_clahe);
//  cv::waitKey();

  delete (hazeRemoval);
}

BOOST_AUTO_TEST_CASE(CalcOverlapPercentInLog_testcase) {

  cv::Mat image_previus = PocologTools::getMatFromStream<
      base::samples::frame::Frame>(video_stream, 0, percent_image_size);

  SingleImageHazeRemoval *hazeRemoval = new CLAHE();
  cv::Mat image_current;
  ImageFeaturesDescriptor image_current_features, image_previus_features;

  std::vector<double> overlaped_areas;

  uint data_size = video_stream->getSize() * 0.05;
  for (int i = 1; i < data_size; i += 5) {
    cv::Mat homografy;

    if (i != 1) {
      image_previus_features = image_current_features;
    } else {
      hazeRemoval->applyHazeRemoval(image_previus);
      image_previus_features = detectorAndDescriptor
          .applyFeatureDetectorAndDescriptorExtractor(image_previus);
    }

    image_current = PocologTools::getMatFromStream<base::samples::frame::Frame>(
        video_stream, i, percent_image_size);

    hazeRemoval->applyHazeRemoval(image_current);
    image_current_features = detectorAndDescriptor
        .applyFeatureDetectorAndDescriptorExtractor(image_current);

    std::vector<cv::DMatch> matches = detectorAndDescriptor
        .applyDescriptorMatcher(image_previus_features.descriptor(),
                                image_current_features.descriptor());

    matches = detectorAndDescriptor.filterMatchersByDistance(matches, 0.2);

    matches = detectorAndDescriptor.filterMatchersByRansacHomografy(
        matches, image_previus_features.key_points(),
        image_current_features.key_points(), 0, &homografy);

    double overlaped_area = image_analysis.calcOverlapPercent(
        image_previus_features.image(), image_current_features.image(),
        homografy);

    overlaped_areas.push_back(overlaped_area);

//    cv::Mat image_matchs;
//    cv::drawMatches(image_previus_features.image(),
//                    image_previus_features.key_points(),
//                    image_current_features.image(),
//                    image_current_features.key_points(), matches, image_matchs,
//                    cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),
//                    std::vector<char>(),
//                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//    cv::imshow("MATCHERS", image_matchs);
//    cv::waitKey();

  }

  cv::Mat1i temp(cv::Mat(overlaped_areas) * 1000);
  cv::Mat1i temp_gt(cv::Mat(gt_overlap_areas) * 1000);

  BOOST_CHECK_EQUAL_COLLECTIONS(temp_gt.begin(), temp_gt.end(), temp.begin(),
                                temp.end());

  delete (hazeRemoval);
}
//
cv::Mat frameConverter(const base::samples::frame::Frame& sample) {
  cv::Mat frame = frame_helper::FrameHelper::convertToCvMat(sample);
  cv::cvtColor(frame, frame, CV_BayerGR2RGB);
  return frame;
}

double dvlVelocityCallBack(const base::samples::RigidBodyState sample) {

  return sample.velocity[0] * sample.velocity[0]
      + sample.velocity[1] * sample.velocity[1]
      + sample.velocity[2] * sample.velocity[2];
}

BOOST_AUTO_TEST_CASE(RelationOverlapAndVelocity_testCase) {
  float data_stream_size_percent = 0.05;
  float skip_frames = 5;
  std::map<int, int> sync_map = PocologTools::syncPocologStreamsByTime<
      base::samples::frame::Frame, base::samples::RigidBodyState>(
      video_stream, dvl_stream, data_stream_size_percent);

  cv::Mat1f relation;

  cv::Mat image_previus = PocologTools::getMatFromStream<
      base::samples::frame::Frame>(video_stream, 0, percent_image_size);

  SingleImageHazeRemoval *hazeRemoval = new CLAHE();
  cv::Mat image_current;
  ImageFeaturesDescriptor image_current_features, image_previus_features;

  uint data_size = video_stream->getSize() * data_stream_size_percent;
  for (int i = 1; i < data_size; i += skip_frames) {
    cv::Mat homografy;

    if (i != 1) {
      image_previus_features = image_current_features;
    } else {
      hazeRemoval->applyHazeRemoval(image_previus);
      image_previus_features = detectorAndDescriptor
          .applyFeatureDetectorAndDescriptorExtractor(image_previus);
    }

    image_current = PocologTools::getMatFromStream<base::samples::frame::Frame>(
        video_stream, i, percent_image_size);

    hazeRemoval->applyHazeRemoval(image_current);
    image_current_features = detectorAndDescriptor
        .applyFeatureDetectorAndDescriptorExtractor(image_current);

    std::vector<cv::DMatch> matches = detectorAndDescriptor
        .applyDescriptorMatcher(image_previus_features.descriptor(),
                                image_current_features.descriptor());

    std::vector<cv::DMatch> temp_matches = detectorAndDescriptor
        .filterMatchersByDistance(matches, 0.2);

    if (temp_matches.size() > 4)
      matches = temp_matches;

    matches = detectorAndDescriptor.filterMatchersByRansacHomografy(
        matches, image_previus_features.key_points(),
        image_current_features.key_points(), 0, &homografy);

    double overlaped_area = image_analysis.calcOverlapPercent(
        image_previus_features.image(), image_current_features.image(),
        homografy);

    // calc the result velocity from DVL
    base::samples::RigidBodyState temp_sample;
    temp_sample = PocologTools::getSampleFromDataStream<
        base::samples::RigidBodyState>(dvl_stream, sync_map.find(i)->second);

    double velocity = dvlVelocityCallBack(temp_sample);

//    std::cout << "Velocity x Overlaped AREA " << velocity << " "
//              << overlaped_area << std::endl;

    cv::Mat1d temp_mat(1, 2);
    temp_mat[0][0] = velocity;
    temp_mat[0][1] = overlaped_area;
    relation.push_back(temp_mat);
  }

  std::string file_name = "skip_frame_" + toStr(skip_frames);
  file_name = file_name + "--log_size_" + toStr(data_stream_size_percent);

  image_analysis.writeMatrixRelation(relation, file_name);
  cv::FileStorage fs_read(file_name + ".yml", cv::FileStorage::READ);
  cv::Mat1f read_mat;
  fs_read["relationMatrix"] >> read_mat;

  cv::Mat1i temp(cv::Mat(read_mat) * 1000);
  cv::Mat1i temp_gt(cv::Mat(gt_relation) * 1000);

  BOOST_CHECK_EQUAL_COLLECTIONS(temp_gt.begin(), temp_gt.end(), temp.begin(),
                                temp.end());

}

//BOOST_AUTO_TEST_CASE(AnalysisImageOverlapInDataStream_testCase) {
//  float data_stream_size_percent = 1.0;
//  float skip_frames = 5;

//  std::map<int, int> sync_map = PocologTools::syncPocologStreamsByTime<
//      base::samples::frame::Frame, base::samples::RigidBodyState>(
//      video_stream, dvl_stream, data_stream_size_percent);
//
//  std::map<int, int> sync_ground_map = PocologTools::syncPocologStreamsByTime<
//      base::samples::frame::Frame, base::samples::RigidBodyState>(
//      video_stream, dvl_ground_stream, data_stream_size_percent);

//  for (skip_frames = 2; skip_frames < 11; skip_frames += 1) {

//    cv::Mat1f relation;
//
//    cv::Mat image_previus = PocologTools::getMatFromStream<
//        base::samples::frame::Frame>(video_stream, 0, percent_image_size);
//
//    SingleImageHazeRemoval *hazeRemoval = new CLAHE();
//    cv::Mat image_current;
//    ImageFeaturesDescriptor image_current_features, image_previus_features;
//
//    uint data_size = video_stream->getSize() * data_stream_size_percent;
//    for (int i = 1; i < data_size; i += 1 + skip_frames) {
//      cv::Mat homografy;
//
//      if (i != 1) {
//        image_previus_features = image_current_features;
//      } else {
//        hazeRemoval->applyHazeRemoval(image_previus);
//        image_previus_features = detectorAndDescriptor
//            .applyFeatureDetectorAndDescriptorExtractor(image_previus);
//      }
//
//      image_current = PocologTools::getMatFromStream<base::samples::frame::Frame>(
//          video_stream, i, percent_image_size);
//
//      hazeRemoval->applyHazeRemoval(image_current);
//      image_current_features = detectorAndDescriptor
//          .applyFeatureDetectorAndDescriptorExtractor(image_current);
//
//      std::vector<cv::DMatch> matches = detectorAndDescriptor
//          .applyDescriptorMatcher(image_previus_features.descriptor(),
//                                  image_current_features.descriptor());
//
//      std::vector<cv::DMatch> temp_matches = detectorAndDescriptor
//          .filterMatchersByDistance(matches, 0.2);
//
//      if (temp_matches.size() > 4)
//        matches = temp_matches;
//
//      matches = detectorAndDescriptor.filterMatchersByRansacHomografy(
//          matches, image_previus_features.key_points(),
//          image_current_features.key_points(), 0, &homografy);
//
//      double overlaped_area = image_analysis.calcOverlapPercent(
//          image_previus_features.image(), image_current_features.image(),
//          homografy);
//
//      // calc the result velocity from DVL
//      base::samples::RigidBodyState sample_velocity, sample_ground_distance;
//      sample_velocity = PocologTools::getSampleFromDataStream<
//          base::samples::RigidBodyState>(dvl_stream, sync_map.find(i)->second);
//
//      sample_ground_distance = PocologTools::getSampleFromDataStream<
//          base::samples::RigidBodyState>(dvl_ground_stream,
//                                         sync_map.find(i)->second);
//
//      double velocity = dvlVelocityCallBack(sample_velocity);
//
//      std::cout << "Velocity, X, Y, Z, z, Overlaped AREA " << velocity << " "
//                << overlaped_area << std::endl;
//
//      cv::Mat1d temp_mat(1, 6);
//      temp_mat[0][0] = velocity;
//      temp_mat[0][1] = sample_velocity.velocity[0];
//      temp_mat[0][2] = sample_velocity.velocity[1];
//      temp_mat[0][3] = sample_velocity.velocity[2];
//      temp_mat[0][4] = sample_ground_distance.position[2];
//      temp_mat[0][5] = overlaped_area;
//
//      relation.push_back(temp_mat);
//      std::cout << "Velocity, X, Y, Z, z, AREA " << temp_mat << std::endl;
//    }
//
//  std::string file_name = "VELOCITY+GROUND_DIST_skip_frame_"
//      + toStr(skip_frames);
//  file_name = file_name + "--log_size_" + toStr(data_stream_size_percent);

//    image_analysis.writeMatrixRelation(relation, file_name);
//  }

//  cv::FileStorage fs_read(file_name + ".yml", cv::FileStorage::READ);
//  cv::Mat1f read_mat;
//  fs_read["relationMatrix"] >> read_mat;
//  std::cout << "read_MAT" << cv::abs(read_mat) << std::endl;

//}
