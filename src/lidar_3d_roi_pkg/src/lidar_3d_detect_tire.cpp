#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class Lidar3d_ROI
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud3d_in;
    ros::Publisher pub_cloud3d_roi;
    ros::Publisher pub_cloud3d_roi_velodyne;
    ros::Publisher pub_cloud2d_projection;
    ros::Publisher pub_projection_image;
    ros::Publisher pub_ai_train_image;
    ros::Publisher pub_tire_centers;     // For publishing tire center markers
    ros::Publisher pub_tire_centers_raw; // For publishing raw tire center data
    ros::Publisher pub_debug_image;      // For publishing debug images
    
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    int image_resolution;
    int no_dilation;
    
    // Parameter for detection
    float min_area;       // Minimum area of a contour to be considered a tire
    float max_area;       // Maximum area of a contour to be considered a tire
    float min_circularity; // Minimum circularity (not used in new approach)
    
    // Debug flags
    bool debug_mode;      // Enable/disable extensive debug output
    
public:
    Lidar3d_ROI() : nh("~")
    {
        nh.param("x_min", x_min, 0.0);
        nh.param("x_max", x_max, 10.0);
        nh.param("y_min", y_min, -10.0);
        nh.param("y_max", y_max, 10.0);
        nh.param("z_min", z_min, -0.2);
        nh.param("z_max", z_max, 0.5);
        nh.param("image_resolution", image_resolution, 100);
        nh.param("no_dilation", no_dilation, 1);
        
        // Add parameters for contour detection
        nh.param("min_area", min_area, 3.0f);  // 작은 크기의 컨투어도 고려
        nh.param("max_area", max_area, 500.0f); // 좀 더 큰 영역도 허용
        nh.param("min_circularity", min_circularity, 0.1f); // 사용하지 않음 (하위 호환성용)
        
        // 디버그 모드 설정
        nh.param("debug_mode", debug_mode, true);
        
        sub_cloud3d_in           = nh.subscribe("/livox/lidar", 1, &Lidar3d_ROI::cloud3d_Callback, this);
        pub_cloud3d_roi          = nh.advertise<sensor_msgs::PointCloud2>("/livox/livox_roi", 1);
        pub_cloud3d_roi_velodyne = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1);
        pub_cloud2d_projection   = nh.advertise<sensor_msgs::PointCloud2>("/livox/tire_detect", 1);
        pub_projection_image     =  nh.advertise<sensor_msgs::Image>("/livox/tire_detect_image", 1);
        pub_ai_train_image       = nh.advertise<sensor_msgs::Image>("/livox/ai_train_image", 1);
        pub_tire_centers         = nh.advertise<visualization_msgs::MarkerArray>("/livox/tire_centers", 1);
        pub_tire_centers_raw     = nh.advertise<geometry_msgs::Point>("/livox/tire_centers_raw", 1);
        pub_debug_image          = nh.advertise<sensor_msgs::Image>("/livox/debug_image", 1);
        
        ROS_INFO("Lidar3d_ROI initialized with parameters:");
        ROS_INFO("ROI: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f]", 
                 x_min, x_max, y_min, y_max, z_min, z_max);
        ROS_INFO("Image resolution: %d pixels/meter", image_resolution);
        ROS_INFO("Dilation operations: %d", no_dilation);
        ROS_INFO("Min contour area: %.2f, Max contour area: %.2f", min_area, max_area);
        ROS_INFO("Debug mode: %s", debug_mode ? "enabled" : "disabled");
    }
    
    void cloud3d_Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
    {
        // ROS 메시지를 PCL 포인트클라우드로 변환
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input_cloud, *cloud); 
        
        // ROI 필터 설정
        pcl::CropBox<pcl::PointXYZI> crop;
        crop.setInputCloud(cloud);
        crop.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
        crop.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
        
        // ROI 적용
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>); 
        crop.filter(*cloud_filtered);
        
        // 필터링된 포인트 클라우드를 ROS 메시지로 변환
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud_filtered, output_cloud); 
        output_cloud.header = input_cloud->header;
        pub_cloud3d_roi.publish(output_cloud);
        
        // velodyne 필터링된 포인트 클라우드를 ROS 메시지로 변환
        sensor_msgs::PointCloud2 output_cloud2;
        pcl::toROSMsg(*cloud_filtered, output_cloud2); 
        output_cloud2.header = input_cloud->header;
        output_cloud2.header.frame_id = "livox_frame"; // frame_id를 livox_frame으로 변경
        pub_cloud3d_roi_velodyne.publish(output_cloud2);
        
        // 3D 포인트 클라우드를 2D로 프로젝션
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d_projection(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_2d_projection->header = cloud_filtered->header;
        cloud_2d_projection->width = cloud_filtered->width;
        cloud_2d_projection->height = 1;
        cloud_2d_projection->is_dense = false;
        cloud_2d_projection->points.resize(cloud_filtered->points.size());
        
        for (size_t i = 0; i < cloud_filtered->points.size(); ++i) 
        {
            cloud_2d_projection->points[i].x = cloud_filtered->points[i].x;
            cloud_2d_projection->points[i].y = cloud_filtered->points[i].y;
            cloud_2d_projection->points[i].z = 0.0;
            cloud_2d_projection->points[i].intensity = cloud_filtered->points[i].intensity;
        }
        
        sensor_msgs::PointCloud2 output_cloud_2d;
        pcl::toROSMsg(*cloud_2d_projection, output_cloud_2d);
        output_cloud_2d.header = input_cloud->header;
        pub_cloud2d_projection.publish(output_cloud_2d);
        
        // 2D 포인트 클라우드를 이미지로 변환하고 타이어 중심 계산
        cv::Mat bird_view = convertPointCloudToImage(cloud_2d_projection, input_cloud->header);
        cv::Mat bird_view1 = convertPointCloudToAIImage(cloud_2d_projection, input_cloud->header);
        
        // 타이어 중심 찾기
        //std::vector<cv::Point2f> tire_centers = findTireCenters(bird_view, input_cloud->header);
    }
    
    cv::Mat convertPointCloudToAIImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std_msgs::Header& header)
    {
        if (cloud->empty()) 
        {
            ROS_WARN("Point cloud is empty, skipping image conversion");
            return cv::Mat();
        }

        float min_x = x_min;
        float max_x = x_max;
        float min_y = y_min;
        float max_y = y_max;
        
        int img_height = static_cast<int>((max_x - min_x) * image_resolution);
        int img_width = static_cast<int>((max_y - min_y) * image_resolution);
        
        img_width = std::max(img_width, 100);  // 최소 너비 보장
        img_height = std::max(img_height, 100);  // 최소 높이 보장
        
        
        ROS_INFO("Image Size: %d x %d", img_width, img_height);
                  
        cv::Mat bird_view = cv::Mat::zeros(img_height, img_width, CV_8UC1);
               

        // 포인트 밀도 계산을 위한 변수
        int point_count  =   0;
        int scale_factor = 255/(z_max - z_min) - 1 ;
        for (const auto& point : cloud->points) 
        {
            int py = static_cast<int>((point.x - min_x) * image_resolution);
            int px = static_cast<int>((point.y - min_y) * image_resolution);
            
            px = img_width - px - 1;
        
            if( (px >= 0 && px < img_width) && (py >= 0 && py < img_height) )
            {
                int pixel_height = (point.z - z_min) * scale_factor;
                pixel_height = (pixel_height >=255) ? 255: pixel_height;
                bird_view.at<uchar>(py, px) = pixel_height; 
                //printf("%d %d %d\n",  px, py,pixel_height);
                point_count++;
            }
        }
        std_msgs::Header img_header = header;
        sensor_msgs::ImagePtr ai_img_msg = cv_bridge::CvImage(img_header, "mono8", bird_view).toImageMsg();
        pub_ai_train_image.publish(ai_img_msg);

        return bird_view;

    
    }
    cv::Mat convertPointCloudToImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std_msgs::Header& header)
    {
        if (cloud->empty()) 
        {
            ROS_WARN("Point cloud is empty, skipping image conversion");
            return cv::Mat();
        }
        
        float min_x = x_min;
        float max_x = x_max;
        float min_y = y_min;
        float max_y = y_max;
        
        int img_height = static_cast<int>((max_x - min_x) * image_resolution);
        int img_width = static_cast<int>((max_y - min_y) * image_resolution);
        
        img_width = std::max(img_width, 100);  // 최소 너비 보장
        img_height = std::max(img_height, 100);  // 최소 높이 보장
        
        ROS_INFO("Image Size: %d x %d", img_width, img_height);
        
        cv::Mat bird_view = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        
        // 포인트 밀도 계산을 위한 변수
        int point_count = 0;
        
        for (const auto& point : cloud->points) {
            int py = static_cast<int>((point.x - min_x) * image_resolution);
            int px = static_cast<int>((point.y - min_y) * image_resolution);
            
            px = img_width - px - 1;
            
            if( (px >= 0 && px < img_width) && (py >= 0 && py < img_height) )
            {
                bird_view.at<uchar>(py, px) = 255;
                point_count++;
            }
        }
        
        // 포인트 밀도 보고
        ROS_INFO("Point density: %d points in %dx%d image (%.2f%%)", 
                 point_count, img_width, img_height, 
                 100.0 * point_count / (img_width * img_height));
        
        // 이미지 전처리 강화
        // 미디언 블러 적용 (노이즈 제거)
        //cv::medianBlur(bird_view, bird_view, 1);
        
        // 더 적극적인 팽창 적용 (타이어 영역 연결)
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::Mat dilated = bird_view.clone();
        
        // 기본 팽창 + 추가 팽창 (보이는 데이터 특성에 맞게 조정)
        int dilate_count = no_dilation + 1;  // 더 강한 팽창을 위해 추가
        
        for (int i = 0; i < dilate_count; i++) {
            cv::dilate(dilated, dilated, element);
        }
        
        // 클로징 연산 추가 (작은 구멍 메우기)
        cv::morphologyEx(dilated, dilated, cv::MORPH_CLOSE, element);
        
        bird_view = dilated;
        
        // 결과 이미지의 흰색 픽셀 수 확인
        int white_pixels = cv::countNonZero(bird_view);
        ROS_INFO("After processing: %d white pixels (%.2f%%)", 
                 white_pixels, 100.0 * white_pixels / (img_width * img_height));
                 
        // 디버그용: 그레이스케일 이미지 발행
        std_msgs::Header img_header = header;
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(img_header, "mono8", bird_view).toImageMsg();
        pub_projection_image.publish(img_msg);


        

        
        // 즉시 시각화를 위한 컬러 이미지도 발행
        cv::Mat color_bird_view;
        cv::cvtColor(bird_view, color_bird_view, cv::COLOR_GRAY2BGR);
        
        // 중심선 그리기 (시험용)
        int center_x = color_bird_view.cols / 2;
        cv::line(color_bird_view, cv::Point(center_x, 0), cv::Point(center_x, color_bird_view.rows), 
                cv::Scalar(0, 255, 255), 3);
        
        // 중심선 이미지 즉시 발행 (디버그용)
        sensor_msgs::ImagePtr center_line_msg = cv_bridge::CvImage(img_header, "bgr8", color_bird_view).toImageMsg();
        pub_debug_image.publish(center_line_msg);
        
        ROS_INFO("이미지 및 중심선 처리 완료. 중심선 좌표: %d", center_x);
        
        return bird_view;
    }
    
    std::vector<cv::Point2f> findTireCenters(const cv::Mat& bird_view, const std_msgs::Header& header)
    {
        // 이미지가 비어있는지 확인
        if (bird_view.empty()) {
            ROS_WARN("Bird view image is empty, skipping tire center detection");
            return std::vector<cv::Point2f>();
        }
        
        // 기본 디버그 이미지 생성 - 원본을 컬러로 변환
        cv::Mat debug_img;
        cv::cvtColor(bird_view, debug_img, cv::COLOR_GRAY2BGR);
        
        // 컨투어를 이용한 접근법으로 변경
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        
        // 이진화 및 컨투어 찾기
        cv::Mat binary = bird_view.clone();
        cv::threshold(binary, binary, 100, 255, cv::THRESH_BINARY);
        
        // 이미지 전처리 - 노이즈 제거
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        
        // 컨투어 찾기
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        ROS_INFO("컨투어 개수: %zu", contours.size());
        
        // 컨투어 필터링 - 너무 작은 컨투어 제거
        std::vector<std::vector<cv::Point>> filtered_contours;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area >= 5.0) {  // 최소 면적 크게 줄임
                filtered_contours.push_back(contour);
            }
        }
        
        ROS_INFO("필터링된 컨투어 개수: %zu", filtered_contours.size());
        
        // 컨투어가 없으면 빈 벡터 반환하면서 디버그 이미지만 출력
        if (filtered_contours.empty()) {
            ROS_WARN("No tire centers detected - no valid contours found");
            
            // 이미지 분석을 위한 추가 디버깅
            ROS_INFO("Image stats - non-zero pixels: %d, size: %dx%d", 
                    cv::countNonZero(bird_view), bird_view.cols, bird_view.rows);
            
            // 중심선만 표시한 디버그 이미지
            int center_x = debug_img.cols / 2;
            // 굵은 중심선 그리기 (빨간색)
            cv::line(debug_img, cv::Point(center_x, 0), cv::Point(center_x, debug_img.rows), 
                    cv::Scalar(0, 0, 255), 5);
            
            // 'CENTER' 텍스트 추가
            cv::putText(debug_img, "CENTER", cv::Point(center_x - 40, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            
            // 디버그 이미지 발행
            sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
            pub_debug_image.publish(debug_msg);
            
            return std::vector<cv::Point2f>();
        }
        
        // 이미지 중앙 위치 계산 (X축 기준 좌우 구분)
        int center_x = bird_view.cols / 2;
        
        // 컨투어 그리기 (디버깅용)
        cv::drawContours(debug_img, filtered_contours, -1, cv::Scalar(0, 255, 0), 2);
        
        // 중심선 먼저 그리기 (디버깅용 - 노란색)
        cv::line(debug_img, cv::Point(center_x, 0), cv::Point(center_x, debug_img.rows), 
                cv::Scalar(0, 255, 255), 3);
                
        // 디버그 이미지에 안내 텍스트 추가
        cv::putText(debug_img, "CENTER LINE", cv::Point(center_x - 50, 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        cv::putText(debug_img, "Contours: " + std::to_string(filtered_contours.size()), 
                    cv::Point(10, debug_img.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 
                    0.5, cv::Scalar(255, 255, 255), 1);
        
        // 모든 컨투어의 중심점 계산
        std::vector<cv::Point2f> centers;
        for (const auto& contour : filtered_contours) {
            cv::Moments m = cv::moments(contour);
            if (m.m00 != 0) {
                cv::Point2f center(m.m10/m.m00, m.m01/m.m00);
                centers.push_back(center);
                
                // 중심점 그리기 (디버깅용 - 빨간색 원)
                cv::circle(debug_img, center, 5, cv::Scalar(0, 0, 255), -1);
            }
        }
        
        // 중심점이 없는 경우 디버그 이미지만 발행하고 종료
        if (centers.empty()) {
            ROS_WARN("컨투어는 있지만 중심점이 계산되지 않음");
            sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
            pub_debug_image.publish(debug_msg);
            return std::vector<cv::Point2f>();
        }
        
        ROS_INFO("중심점 개수: %zu", centers.size());
        
        // 컨투어 기반 중심점을 실제 좌표로 변환하고 왼쪽/오른쪽 구분
        std::vector<cv::Point2f> tire_centers;
        std::vector<cv::Point2f> left_centers, right_centers;
        
        // 이미지 중앙 위치 계산 (X축 기준 좌우 구분을 위해)
        center_x = bird_view.cols / 2;
        
        // 디버깅을 위한 이미지 중앙선 표시 - 더 두껍고 밝은 색상으로
        cv::line(debug_img, cv::Point(center_x, 0), cv::Point(center_x, bird_view.rows), 
                 cv::Scalar(0, 255, 255), 4);  // 두께를 2에서 4로 증가, 밝은 노란색
                 
        // 추가: 중앙선 텍스트 표시
        cv::putText(debug_img, "CENTER", cv::Point(center_x - 30, 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        for (const auto& center : centers) {
            // 이미지 좌표를 실제 좌표로 변환
            float img_x = center.x;
            float img_y = center.y;
            
            // X 좌표는 원래대로 변환
            float real_y = y_max - img_x / image_resolution;
            // Y 좌표는 이미지 좌표계와 실제 좌표계가 다르므로 변환
            float real_x = img_y / image_resolution + x_min;
            
            // 왼쪽/오른쪽 구분 (이미지 x 좌표에 따라)
            if (img_x < center_x) {
                left_centers.push_back(cv::Point2f(real_x, real_y));
                // 디버깅을 위한 왼쪽 중심점 표시 (빨간색)
                cv::circle(debug_img, center, 8, cv::Scalar(0, 0, 255), -1);
            } else {
                right_centers.push_back(cv::Point2f(real_x, real_y));
                // 디버깅을 위한 오른쪽 중심점 표시 (파란색)
                cv::circle(debug_img, center, 8, cv::Scalar(255, 0, 0), -1);
            }
            
            // 모든 중심점 저장
            tire_centers.push_back(cv::Point2f(real_x, real_y));
        }
        
        // 디버깅을 위한 이미지 중앙선 표시
        cv::line(debug_img, cv::Point(center_x, 0), cv::Point(center_x, bird_view.rows), 
                 cv::Scalar(0, 255, 255), 2);
        
        // 왼쪽 타이어 중심점 계산
        cv::Point2f left_center(0, 0);
        if (!left_centers.empty()) {
            for (const auto& center : left_centers) {
                left_center.x += center.x;
                left_center.y += center.y;
            }
            left_center.x /= left_centers.size();
            left_center.y /= left_centers.size();
            
            ROS_INFO("Left tire center: (%.2f, %.2f)", left_center.x, left_center.y);
        }
        
        // 오른쪽 타이어 중심점 계산
        cv::Point2f right_center(0, 0);
        if (!right_centers.empty()) {
            for (const auto& center : right_centers) {
                right_center.x += center.x;
                right_center.y += center.y;
            }
            right_center.x /= right_centers.size();
            right_center.y /= right_centers.size();
            
            ROS_INFO("Right tire center: (%.2f, %.2f)", right_center.x, right_center.y);
        }
        
        // 타이어 중심에 붉은색 라인 그리기
        cv::Mat center_img = bird_view.clone();
        // 그레이스케일 이미지를 컬러로 변환
        cv::cvtColor(center_img, center_img, cv::COLOR_GRAY2BGR);
        
        // 기존에 계산된 center_x 변수 재사용 (재선언하지 않음)
        cv::line(center_img, cv::Point(center_x, 0), cv::Point(center_x, center_img.rows), 
                 cv::Scalar(0, 255, 255), 5);  // 두꺼운 노란색 선
                 
        cv::putText(center_img, "CENTER LINE", cv::Point(center_x - 50, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        
        // 왼쪽 타이어 중심에 십자가 그리기
        if (left_center.x != 0 || left_center.y != 0) {
            // 이미지 좌표로 변환
            int img_y = static_cast<int>((left_center.x - x_min) * image_resolution);
            int img_x = static_cast<int>((y_max - left_center.y) * image_resolution);
            
            // 십자가 그리기 (빨간색)
            int line_length = 20;  // 십자가 길이
            int line_thickness = 3; // 라인 두께 증가
            cv::Scalar line_color(0, 0, 255); // BGR 형식 (빨간색)
            
            // 수평선
            cv::line(center_img, 
                    cv::Point(std::max(0, img_x - line_length), img_y), 
                    cv::Point(std::min(center_img.cols - 1, img_x + line_length), img_y), 
                    line_color, line_thickness);
            
            // 수직선
            cv::line(center_img, 
                    cv::Point(img_x, std::max(0, img_y - line_length)), 
                    cv::Point(img_x, std::min(center_img.rows - 1, img_y + line_length)), 
                    line_color, line_thickness);
                    
            // 텍스트 추가 (선택사항) - 더 두껍고 크게
            cv::putText(center_img, "Left", cv::Point(img_x + 5, img_y - 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, line_color, 2);
        }
        
        // 오른쪽 타이어 중심에 십자가 그리기
        if (right_center.x != 0 || right_center.y != 0) {
            // 이미지 좌표로 변환
            int img_y = static_cast<int>((right_center.x - x_min) * image_resolution);
            int img_x = static_cast<int>((y_max - right_center.y) * image_resolution);
            
            // 십자가 그리기 (빨간색)
            int line_length = 20;  // 십자가 길이
            int line_thickness = 3; // 라인 두께 증가
            cv::Scalar line_color(0, 0, 255); // BGR 형식 (빨간색)
            
            // 수평선
            cv::line(center_img, 
                    cv::Point(std::max(0, img_x - line_length), img_y), 
                    cv::Point(std::min(center_img.cols - 1, img_x + line_length), img_y), 
                    line_color, line_thickness);
            
            // 수직선
            cv::line(center_img, 
                    cv::Point(img_x, std::max(0, img_y - line_length)), 
                    cv::Point(img_x, std::min(center_img.rows - 1, img_y + line_length)), 
                    line_color, line_thickness);
                    
            // 텍스트 추가 (선택사항) - 더 두껍고 크게
            cv::putText(center_img, "Right", cv::Point(img_x + 5, img_y - 10), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, line_color, 2);
        }
        
        // 디버그 이미지 발행 (타이어 중심 십자가 포함)
        sensor_msgs::ImagePtr center_msg = cv_bridge::CvImage(header, "bgr8", center_img).toImageMsg();
        pub_debug_image.publish(center_msg);
        
        // 마커 생성 및 발행
        publishTireCentersMarker(left_center, right_center, header);
        
        return tire_centers;
    }
    
    void publishTireCentersMarker(const cv::Point2f& left_center, const cv::Point2f& right_center, 
                                  const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray marker_array;
        
        // 왼쪽 타이어 중심 마커
        if (left_center.x != 0 || left_center.y != 0) {
            visualization_msgs::Marker left_marker;
            left_marker.header = header;
            left_marker.header.frame_id = "livox_frame";  // frame_id를 livox_frame으로 변경
            left_marker.ns = "tire_centers";
            left_marker.id = 0;
            left_marker.type = visualization_msgs::Marker::SPHERE;
            left_marker.action = visualization_msgs::Marker::ADD;
            left_marker.pose.position.x = left_center.x;
            left_marker.pose.position.y = left_center.y;
            left_marker.pose.position.z = 0.0;
            left_marker.pose.orientation.w = 1.0;
            left_marker.scale.x = 0.3;
            left_marker.scale.y = 0.3;
            left_marker.scale.z = 0.3;
            left_marker.color.r = 1.0;
            left_marker.color.g = 0.0;
            left_marker.color.b = 0.0;
            left_marker.color.a = 1.0;
            left_marker.lifetime = ros::Duration(0.1);  // 100ms 동안 표시
            
            marker_array.markers.push_back(left_marker);
            
            // 왼쪽 타이어 중심점 발행
            geometry_msgs::Point left_point;
            left_point.x = left_center.x;
            left_point.y = left_center.y;
            left_point.z = 0.0;
            pub_tire_centers_raw.publish(left_point);
        }
        
        // 오른쪽 타이어 중심 마커
        if (right_center.x != 0 || right_center.y != 0) {
            visualization_msgs::Marker right_marker;
            right_marker.header = header;
            right_marker.header.frame_id = "livox_frame";  // frame_id를 livox_frame으로 변경
            right_marker.ns = "tire_centers";
            right_marker.id = 1;
            right_marker.type = visualization_msgs::Marker::SPHERE;
            right_marker.action = visualization_msgs::Marker::ADD;
            right_marker.pose.position.x = right_center.x;
            right_marker.pose.position.y = right_center.y;
            right_marker.pose.position.z = 0.0;
            right_marker.pose.orientation.w = 1.0;
            right_marker.scale.x = 0.3;
            right_marker.scale.y = 0.3;
            right_marker.scale.z = 0.3;
            right_marker.color.r = 0.0;
            right_marker.color.g = 0.0;
            right_marker.color.b = 1.0;
            right_marker.color.a = 1.0;
            right_marker.lifetime = ros::Duration(0.1);  // 100ms 동안 표시
            
            marker_array.markers.push_back(right_marker);
            
            // 오른쪽 타이어 중심점 발행
            geometry_msgs::Point right_point;
            right_point.x = right_center.x;
            right_point.y = right_center.y;
            right_point.z = 0.0;
            pub_tire_centers_raw.publish(right_point);
        }
        
        if (!marker_array.markers.empty()) {
            pub_tire_centers.publish(marker_array);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_3d_detect_tire_node");
    Lidar3d_ROI lidar3d_roi; 
    ros::spin();
    return 0;
}
