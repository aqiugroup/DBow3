// First step of creating a vocabulary is extracting features from a set of images. We save them to a file for next step
#include <iostream>
#include <vector>

// DBoW3
#include "DBoW3.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef USE_CONTRIB
#    include <opencv2/xfeatures2d.hpp>
#    include <opencv2/xfeatures2d/nonfree.hpp>

#endif
#include "DescManip.h"
#include "image_processor.h"
#include "time_utils.h"

using namespace DBoW3;
using namespace std;
using namespace cv;

msckf_vio::ImageProcessor vio_msckf_frontend_;

// command line parser
class CmdLineParser {
    int    argc;
    char** argv;

  public:
    CmdLineParser(int _argc, char** _argv) : argc(_argc), argv(_argv) {}
    bool operator[](string param)
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        return (idx != -1);
    }
    string operator()(string param, string defvalue = "-1")
    {
        int idx = -1;
        for (int i = 0; i < argc && idx == -1; i++)
            if (string(argv[i]) == param)
                idx = i;
        if (idx == -1)
            return defvalue;
        else
            return (argv[idx + 1]);
    }
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait()
{
    cout << endl << "Press enter to continue" << endl;
    getchar();
}

void LoadImagesMono(const string& strFile, vector<string>& vLeftImageFilenames, vector<double>& vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());
    if (!f.is_open()) {
        cerr << "ERROR::: Output File " << strFile.c_str() << " not exists!!!!!" << endl;
        exit(0);
    }
    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            // ss >> t;
            // vTimestamps.push_back(t);
            ss >> sRGB;
            if (sRGB.find("left") != std::string::npos) {
                vLeftImageFilenames.push_back(sRGB);
            }
            // else if (sRGB.find("right") != std::string::npos) {
            //     vRightImageFilenames.push_back(sRGB);
            // }

            int         filename_index  = sRGB.find_last_of("/");
            int         timestamp_index = sRGB.find_last_of("_");
            std::string t_str           = sRGB.substr(filename_index + 1, timestamp_index - filename_index - 1);
            double      t_l             = std::stod(t_str);
            t                           = t_l / 1.0e9;
            vTimestamps.push_back(t);
        }
    }
    f.close();
}

void LoadImagesStereo(const string&   strFile,
                      vector<string>& vLeftImageFilenames,
                      vector<string>& vRightImageFilenames,
                      vector<double>& vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());
    if (!f.is_open()) {
        cerr << "ERROR::: Output File " << strFile.c_str() << " not exists!!!!!" << endl;
        exit(0);
    }
    int left_cnt  = 0;
    int right_cnt = 0;
    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            // ss >> t;
            // vTimestamps.push_back(t);
            ss >> sRGB;
            if (sRGB.find("left") != std::string::npos) {
                vLeftImageFilenames.push_back(sRGB);
                left_cnt++;
            }
            else if (sRGB.find("right") != std::string::npos) {
                vRightImageFilenames.push_back(sRGB);
                right_cnt++;
            }

            if (left_cnt == 1 && right_cnt == 1) {
                left_cnt                    = 0;
                right_cnt                   = 0;
                int         filename_index  = sRGB.find_last_of("/");
                int         timestamp_index = sRGB.find_last_of("_");
                std::string t_str           = sRGB.substr(filename_index + 1, timestamp_index - filename_index - 1);
                double      t_l             = std::stod(t_str);
                t                           = t_l / 1.0e9;
                vTimestamps.push_back(t);
            }
        }
    }
    f.close();
}

vector<string> readImagePaths(int argc, char** argv, int start)
{
    vector<string> paths;
    for (int i = start; i < argc; i++)
        paths.push_back(argv[i]);
    return paths;
}

cv::Mat   erosion_dst, dilation_dst;
int       erosion_elem    = 0;
int       erosion_size    = 2;
int       dilation_elem   = 0;
int       dilation_size   = 2;
int const max_elem        = 2;
int const max_kernel_size = 21;

void Erosion(cv::Mat& src)
{
    int erosion_type = 0;
    if (erosion_elem == 0) {
        erosion_type = cv::MORPH_RECT;
    }
    else if (erosion_elem == 1) {
        erosion_type = cv::MORPH_CROSS;
    }
    else if (erosion_elem == 2) {
        erosion_type = cv::MORPH_ELLIPSE;
    }
    cv::Mat element = cv::getStructuringElement(erosion_type, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));
    cv::erode(src, erosion_dst, element);
    cv::imshow("Erosion Demo", erosion_dst);
    cv::waitKey(50000);
}

void Dilation(cv::Mat& src)
{
    int dilation_type = 0;
    if (dilation_elem == 0) {
        dilation_type = MORPH_RECT;
    }
    else if (dilation_elem == 1) {
        dilation_type = MORPH_CROSS;
    }
    else if (dilation_elem == 2) {
        dilation_type = MORPH_ELLIPSE;
    }
    Mat element = getStructuringElement(dilation_type, Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        Point(dilation_size, dilation_size));
    dilate(src, dilation_dst, element);
    imshow("Dilation Demo", dilation_dst);
}

int         threshold_value  = 250;
int         threshold_type   = 0;
int const   max_value        = 255;
int const   max_type         = 4;
int const   max_binary_value = 255;
Mat         src, src_gray, dst;
const char* window_name = "Threshold Demo";
const char* trackbar_type =
    "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
const char* trackbar_value = "Value";

static void Threshold_Demo(int, void*)
{
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
    */
    threshold(src_gray, dst, threshold_value, max_binary_value, threshold_type);
    imshow(window_name, dst);

    waitKey();
    // Erosion(dst);
    Dilation(dst);
    imshow(window_name, dst);
}

void thresholdGray(cv::Mat& src, cv::Mat& mask)
{
    int       threshold_value  = 250;
    int       threshold_type   = 0;
    int const max_value        = 255;
    int const max_type         = 4;
    int const max_binary_value = 255;
    /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
    */
    threshold(src, mask, threshold_value, max_binary_value, threshold_type);
}

void thresholdGrayManual(cv::Mat& src, cv::Mat& mask)
{
    int const min_binary_value = 250;
    int const max_binary_value = 255;
    // std::stringstream ss;
    int m = 0;
    int n = 0;
    for (int y = 0; y < src.rows; y++) {
        for (int x = 0; x < src.cols; x++) {
            int gray = src.at<uchar>(y, x);
            // ss << gray << " ";
            if ((max_binary_value >= gray) && (gray >= min_binary_value)) {
                mask.at<uchar>(y, x) = 255;
                m++;
            }
            else {
                n++;
                mask.at<uchar>(y, x) = 0;
            }
        }
    }
    // std::cout << ss.str() << " " << m << " " << n << std::endl;
}

void dilation(cv::Mat& mask)
{
    int dilation_elem = 0;
    int dilation_size = 5;
    int dilation_type = 0;

    if (dilation_elem == 0) {
        dilation_type = MORPH_RECT;
    }
    else if (dilation_elem == 1) {
        dilation_type = MORPH_CROSS;
    }
    else if (dilation_elem == 2) {
        dilation_type = MORPH_ELLIPSE;
    }
    Mat element = getStructuringElement(dilation_type, Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                        Point(dilation_size, dilation_size));
    dilate(mask, mask, element);
}

vector<cv::Mat> loadFeatures(std::vector<string>& path_to_left_images,
                             std::vector<string>& path_to_right_images,
                             string               descriptor = "") throw(std::exception)
{
    bool openLog = false;
    // select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor == "orb")
        fdetector = cv::ORB::create();
    else if (descriptor == "brisk")
        fdetector = cv::BRISK::create();
#ifdef OPENCV_VERSION_3
    else if (descriptor == "akaze")
        fdetector = cv::AKAZE::create();
#endif
#ifdef USE_CONTRIB
    else if (descriptor == "surf")
        fdetector = cv::xfeatures2d::SURF::create(400, 4, 2, EXTENDED_SURF);
#endif

    else
        throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    vector<cv::Mat> features;

    bool use_stereo = false;
    if (use_stereo == true && path_to_left_images.size() != path_to_right_images.size()) {
        std::cerr << "left image size != right image size" << std::endl;
        return features;
    }

    cout << "Extracting   features..." << descriptor.c_str() << endl;
    int valid_image_cnt = 0;
    for (size_t i = 0; i < path_to_left_images.size(); ++i) {
        vector<cv::KeyPoint> keypoints;
        cv::Mat              descriptors;
        if (openLog)
            cout << "reading image: " << path_to_left_images[i] << endl;
        cv::Mat imageL = cv::imread(path_to_left_images[i], 0);
        if (imageL.empty())
            throw std::runtime_error("Could not open imageL " + path_to_left_images[i]);
        if (openLog)
            cout << "extracting features " << imageL.rows << " " << imageL.cols << endl;

        bool     blurJudge = false;
        float    blurStdTh = 4.5;
        double   m1 = 0, sd1 = 0;
        TicTocV2 time_blur(openLog);
        if (blurJudge) {
            // 记录方差
            cv::Mat lap_image;
            cv::Mat tmp_m1, tmp_sd1;  //用来存储均值和方差

            cv::Laplacian(imageL, lap_image, CV_32FC1);
            lap_image = cv::abs(lap_image);
            cv::meanStdDev(lap_image, tmp_m1, tmp_sd1);
            m1  = tmp_m1.at<double>(0, 0);   //均值
            sd1 = tmp_sd1.at<double>(0, 0);  //标准差
            if (sd1 < blurStdTh) {
                std::cout << "!!!!!!!! maybe blur " << i << " valid " << valid_image_cnt << " std " << sd1 << std::endl;
                continue;
            }
        }
        time_blur.toc("blurJudge");

        // Erosion(imageL);

        // src_gray = imageL;
        // namedWindow(window_name, WINDOW_AUTOSIZE);  // Create a window to display results
        // createTrackbar(trackbar_type, window_name, &threshold_type, max_type,
        //                Threshold_Demo);  // Create a Trackbar to choose type of Threshold
        // createTrackbar(trackbar_value, window_name, &threshold_value, max_value,
        //                Threshold_Demo);  // Create a Trackbar to choose Threshold value
        // Threshold_Demo(0, 0);            // Call the function to initialize
        // waitKey();
        TicTocV2 timeElapse(openLog);
        timeElapse.tic();
        cv::Mat mask(imageL.rows, imageL.cols, CV_8UC1);
        thresholdGray(imageL, mask);
        // thresholdGrayManual(imageL, mask);
        timeElapse.toc("thresholdGray");

        timeElapse.tic();
        dilation(mask);
        timeElapse.toc("dilation");

        timeElapse.tic();
        bitwise_not(mask, mask);
        timeElapse.toc("bitwise_not");

        // imshow(window_name, mask);
        // waitKey();

        cv::Mat imageR;
        double  t(0.0);
        if (use_stereo) {
            imageR = cv::imread(path_to_right_images[i], 0);
            if (imageR.empty()) {
                throw std::runtime_error("Could not open imageR " + path_to_right_images[i]);
            }
            cout << "extracting features " << imageR.rows << " " << imageR.cols << endl;
        }

        TicTocV2 detectAndCompute_t(openLog);
        detectAndCompute_t.tic();
        fdetector->detectAndCompute(imageL, mask, keypoints, descriptors);
        // fdetector->detectAndCompute(imageL, cv::Mat(), keypoints, descriptors);
        detectAndCompute_t.toc("detectAndCompute");

        vector<int> inlier_markers;
        // vector<pair<double, std::vector<Eigen::Matrix<double, 5, 1>>>> tempmsckf_feature;
        if (use_stereo) {  // vio_msckf_frontend_.setFirstImgFlag(true);
            // tempmsckf_feature = vio_msckf_frontend_.stereoCallback(imageL, imageR, t);

            vio_msckf_frontend_.putInGrid(imageL, keypoints, inlier_markers);
            vector<cv::KeyPoint> keypointsGrid;
            cv::Mat              descriptorsGrid((int)inlier_markers.size(), 32, CV_8UC1);
            int                  inlier_size = inlier_markers.size();
            std::cout << "descriptors row " << descriptors.rows << " cols " << descriptors.cols << " inlier_size "
                      << inlier_size << std::endl;
            for (size_t i = 0; i < inlier_size; i++) {
                int index = inlier_markers[i];
                keypointsGrid.push_back(keypoints[index]);
                descriptors.row(index).copyTo(descriptorsGrid.row(i));
            }
            if (inlier_size > 0) {
                keypoints   = keypointsGrid;
                descriptors = descriptorsGrid;
            }
        }

        // imshow(window_name, imageL);
        // waitKey();

        // // debug : resize image
        // cv::Mat image2;
        // cv::resize(imageL, image2, cv::Size(cv::Size2i(960, 540)));
        // cout << "imageL size : " << image2.rows << " " << image2.cols << endl;
        // detectAndCompute_t.tic();
        // fdetector->detectAndCompute(image2, cv::Mat(), keypoints, descriptors);
        // detectAndCompute_t.toc("detectAndCompute");
        // // debug : split
        // detectAndCompute_t.tic();
        // vector<cv::KeyPoint> keypoints2;
        // cv::Mat              descriptors2;
        // fdetector->detect(image2, keypoints2, cv::Mat());
        // detectAndCompute_t.toc("detect");
        // detectAndCompute_t.tic();
        // fdetector->compute(image2, keypoints2, descriptors2);
        // detectAndCompute_t.toc("compute");
        if (openLog)
            std::cout << "descriptors row " << descriptors.rows << " cols " << descriptors.cols << " inlier_size "
                      << keypoints.size() << std::endl;

        features.push_back(descriptors);
        if (openLog)
            cout << "done detecting features" << endl;

        // debug : save imageL
        bool debug_save_imageL = true;
        if (debug_save_imageL) {
            cv::Mat     image_save        = cv::imread(path_to_left_images[i]);
            int         MATCH_IMAGE_SCALE = 1;
            int         index             = path_to_left_images[i].find_last_of("/");
            std::string folderPath        = path_to_left_images[i].substr(0, index);
            std::string filename          = path_to_left_images[i].substr(index + 1, -1);

            int         index2     = path_to_left_images[i].find_last_of(".");
            std::string imageName  = path_to_left_images[i].substr(index + 1, index2 - index - 1);
            std::string extendName = path_to_left_images[i].substr(index2 + 1, -1);

            std::string save_image_with_feat = folderPath + "/" + imageName + "_feature.png";
            for (size_t i = 0; i < keypoints.size(); i++) {
                cv::circle(image_save, keypoints[i].pt, 2, cv::Scalar(0, 255, 0), 2);
                // cv::putText(image_save, to_string(keypoints[i].response), keypoints[i].pt, CV_FONT_HERSHEY_SIMPLEX,
                // 0.8,
                //             cv::Scalar(255, 0, 255), 2);
                // cout << "i " << i << " xy " << keypoints[i].pt.x << " " << keypoints[i].pt.y << " score "
                //      << keypoints[i].response << endl;
            }
            // cv::drawKeypoints(image_save, keypoints, image_save, cv::Scalar(0, 255, 0), 0);

            int        text_height_pos = 20;
            double     text_scale      = 0.8 * MATCH_IMAGE_SCALE;
            int        text_thickness  = 2 * MATCH_IMAGE_SCALE;
            cv::Scalar text_color      = cv::Scalar(255, 0, 255);
            string     display_str = "Frame: " + to_string(keypoints.size()) + "_" + to_string(descriptors.rows) + "_" +
                                 to_string(m1) + "_" + to_string(sd1);
            // opencv 3
            // cv::putText(image_save, display_str, cv::Point2f(5, text_height_pos), CV_FONT_HERSHEY_SIMPLEX,
            // text_scale,
            //             text_color, text_thickness);
            // opencv 4
            cv::putText(image_save, display_str, cv::Point2f(5, text_height_pos), FONT_HERSHEY_SIMPLEX, text_scale,
                        text_color, text_thickness);
            // cout << "!!!!!!!index : " << index << " index2 " << index2 << endl;
            // cout << "path_to_left_images[i] : " << path_to_left_images[i].c_str() << endl;
            // cout << "folderPath : " << folderPath.c_str() << endl;
            // cout << "imageName : " << imageName.c_str() << endl;
            // cout << "extendName : " << extendName.c_str() << endl;

            int        grid_row    = 4;
            int        grid_col    = 5;
            static int grid_height = image_save.rows / grid_row;
            static int grid_width  = image_save.cols / grid_col;
            int        img_width   = image_save.cols;
            int        img_height  = image_save.rows;
            for (int i = 1; i < grid_row; ++i) {
                Point pt1(0, i * grid_height);
                Point pt2(img_width, i * grid_height);
                line(image_save, pt1, pt2, Scalar(255, 0, 0));
            }
            for (int i = 1; i < grid_col; ++i) {
                Point pt1(i * grid_width, 0);
                Point pt2(i * grid_width, img_height);
                line(image_save, pt1, pt2, Scalar(255, 0, 0));
            }

            if (openLog)
                cout << "save feature image : " << save_image_with_feat.c_str() << endl;
            cv::imwrite(save_image_with_feat, image_save);
        }

        valid_image_cnt++;
    }
    cout << "all image size " << path_to_left_images.size() << " valid image size " << valid_image_cnt << endl;

    return features;
}

// ----------------------------------------------------------------------------
void saveToFile(string filename, const vector<cv::Mat>& features)
{
    // test it is not created
    // std::ifstream ifile(filename);
    // if (ifile.is_open()) {
    //     cerr << "ERROR::: input File " << filename << " already exists!!!!!" << endl;
    //     exit(0);
    // }
    std::ofstream ofile(filename);
    if (!ofile.is_open()) {
        cerr << "could not open output file" << endl;
        exit(0);
    }
    uint32_t size = features.size();
    ofile.write((char*)&size, sizeof(size));
    for (auto& f : features) {
        if (!f.isContinuous()) {
            cerr << "Matrices should be continuous" << endl;
            exit(0);
        }
        uint32_t aux = f.cols;
        ofile.write((char*)&aux, sizeof(aux));
        aux = f.rows;
        ofile.write((char*)&aux, sizeof(aux));
        aux = f.type();
        ofile.write((char*)&aux, sizeof(aux));
        ofile.write((char*)f.ptr<uchar>(0), f.total() * f.elemSize());
    }
}

// ----------------------------------------------------------------------------

int main(int argc, char** argv)
{
    try {
        CmdLineParser cml(argc, argv);
        if (cml["-h"] || argc == 1) {
            cerr << "Usage:  descriptor_name output image0 image1 ... \n\t "
                    "descriptors:brisk,surf,orb(default),akaze(only if using opencv 3)"
                 << endl;
            return -1;
        }

        string descriptor = argv[1];
        string output     = argv[2];

        // auto            images   = readImagePaths(argc, argv, 3);
        vector<string> vLeftImageFilenames;
        vector<string> vRightImageFilenames;
        vector<double> vTimestamps;
        // /media/qiuzc/Document/4_data/kimera_data/true_data/47_floor/220114/debug/2202208001/image/all_left_images.txt
        string all_left_images = "/media/qiuzc/Document/4_data/kimera_data/true_data/47_floor/220114/debug/2202208001/"
                                 //  "image/all_left_images.txt";
                                 "image/all_left_images_test1.txt";
        if (argc >= 4) {
            all_left_images = argv[3];
            printf("input image dir: %s\n", all_left_images.c_str());
            fflush(stdout);  // Will now print everything in the stdout buffer
        }

        // LoadImagesMono(all_left_images, vLeftImageFilenames, vTimestamps);
        LoadImagesStereo(all_left_images, vLeftImageFilenames, vRightImageFilenames, vTimestamps);

        vector<cv::Mat> features = loadFeatures(vLeftImageFilenames, vRightImageFilenames, descriptor);

        // save features to file
        saveToFile(argv[2], features);
    }
    catch (std::exception& ex) {
        cerr << ex.what() << endl;
    }

    return 0;
}