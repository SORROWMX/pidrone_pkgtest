/*
 * Optical Flow node for INAV
 * Based on code by Copter Express Technologies
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 */

#include <vector>
#include <cmath>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>

using cv::Mat;

class OpticalFlow : public nodelet::Nodelet
{
public:
    OpticalFlow():
        camera_matrix_(3, 3, CV_64F)
    {}

private:
    ros::Publisher inav_flow_pub_, shift_pub_, velo_pub_;
    ros::Time prev_stamp_;
    image_transport::CameraSubscriber img_sub_;
    image_transport::Publisher img_pub_;
    int roi_px_;
    double roi_rad_; // Добавляем параметр для расчета ROI в радианах
    cv::Rect roi_;
    Mat hann_;
    Mat prev_, curr_;
    Mat camera_matrix_, dist_coeffs_;
    std::string frame_id_;
    double max_frame_age_; // Максимальный возраст кадра в секундах

    void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nh_priv = getPrivateNodeHandle();
        image_transport::ImageTransport it(nh);
        image_transport::ImageTransport it_priv(nh_priv);

        // Получаем параметры
        roi_px_ = nh_priv.param("roi", 128);
        roi_rad_ = nh_priv.param("roi_rad", 0.0); // Параметр для расчета ROI в радианах
        frame_id_ = nh_priv.param<std::string>("frame_id", "base_link");
        max_frame_age_ = nh_priv.param("max_frame_age", 0.1); // Максимальный возраст кадра (по умолчанию 100 мс)

        // Публикаторы
        img_pub_ = it_priv.advertise("debug", 1);
        inav_flow_pub_ = nh.advertise<std_msgs::Float32MultiArray>("optical_flow/inav_data", 1);
        shift_pub_ = nh_priv.advertise<geometry_msgs::Vector3Stamped>("shift", 1);
        velo_pub_ = nh_priv.advertise<geometry_msgs::TwistStamped>("angular_velocity", 1);

        // Подписчики
        img_sub_ = it.subscribeCamera("image_raw", 1, &OpticalFlow::flow, this);

        NODELET_INFO("Optical Flow initialized for INAV");
        NODELET_INFO("Waiting for images on topic: %s", img_sub_.getTopic().c_str());
        NODELET_INFO("ROI size: %d px, ROI radius: %.3f rad", roi_px_, roi_rad_);
        NODELET_INFO("Max frame age: %.3f sec", max_frame_age_);
    }

    void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                camera_matrix_.at<double>(i, j) = cinfo->K[3 * i + j];
            }
        }
        dist_coeffs_ = cv::Mat(cinfo->D, true);
    }

    void drawFlow(Mat& frame, double x, double y, double quality) const
    {
        double brightness = (1 - quality) * 25;
        cv::Scalar color(brightness, brightness, brightness);
        double radius = std::sqrt(x * x + y * y);

        // draw a circle and line indicating the shift direction...
        cv::Point center(frame.cols >> 1, frame.rows >> 1);
        cv::circle(frame, center, (int)(radius*5), color, 3, cv::LINE_AA);
        cv::line(frame, center, cv::Point(center.x + (int)(x*5), center.y + (int)(y*5)), color, 3, cv::LINE_AA);
    }

    void flow(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo)
    {
        try {
            parseCameraInfo(cinfo);

            // Конвертируем изображение
            auto img = cv_bridge::toCvShare(msg, "mono8")->image;

            // Применяем ROI если нужно
            if (roi_.width == 0) { // ROI еще не рассчитан
                // Рассчитываем ROI с учетом искажений камеры
                if (roi_rad_ > 0) {
                    // Создаем 3D точки в пространстве, соответствующие углу обзора roi_rad_
                    std::vector<cv::Point3f> object_points = {
                        cv::Point3f(-sin(roi_rad_ / 2), -sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
                        cv::Point3f(sin(roi_rad_ / 2), sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
                    };

                    // Проецируем 3D точки на плоскость изображения с учетом параметров камеры
                    std::vector<double> vec { 0, 0, 0 };
                    std::vector<cv::Point2f> img_points;
                    cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points);

                    // Создаем ROI на основе проецированных точек
                    roi_ = cv::Rect(
                        cv::Point2i(round(img_points[0].x), round(img_points[0].y)),
                        cv::Point2i(round(img_points[1].x), round(img_points[1].y))
                    );
                    
                    NODELET_INFO("ROI based on projection: %d %d - %d %d", 
                        roi_.tl().x, roi_.tl().y, roi_.br().x, roi_.br().y);
                } else if (roi_px_ > 0) {
                    // Используем фиксированный размер ROI, центрированный в изображении
                    int center_x = msg->width / 2;
                    int center_y = msg->height / 2;
                    int half_roi = roi_px_ / 2;
                    roi_ = cv::Rect(center_x - half_roi, center_y - half_roi, roi_px_, roi_px_);
                }
            }
            
            // Проверяем, что ROI находится в границах изображения
            if (roi_.width > 0 && roi_.height > 0) {
                cv::Rect img_rect(0, 0, img.cols, img.rows);
                roi_ = roi_ & img_rect;
                
                if (roi_.width > 0 && roi_.height > 0) {
                    img = img(roi_);
                }
            }

            // Конвертируем в float для фазовой корреляции
            img.convertTo(curr_, CV_32F);

            // Проверка на устаревший предыдущий кадр
            if (prev_.empty() || (msg->header.stamp - prev_stamp_).toSec() > max_frame_age_) {
                prev_ = curr_.clone();
                prev_stamp_ = msg->header.stamp;
                cv::createHanningWindow(hann_, curr_.size(), CV_32F);
                return;
            }

            // Вычисляем фазовую корреляцию
            double response;
            cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response);

            // Публикуем сдвиг в пикселях
            geometry_msgs::Vector3Stamped shift_vec;
            shift_vec.header.stamp = msg->header.stamp;
            shift_vec.header.frame_id = msg->header.frame_id;
            shift_vec.vector.x = shift.x;
            shift_vec.vector.y = shift.y;
            shift_pub_.publish(shift_vec);

            // Улучшенная обработка искажений линз
            // Вычисляем поток в радианах с учетом искажений камеры
            double flow_center_x = img.cols / 2;
            double flow_center_y = img.rows / 2;
            
            // Создаем точку сдвига с центром в середине ROI
            cv::Point2d shift_point(shift.x + flow_center_x, shift.y + flow_center_y);
            
            // Создаем массивы для undistortPoints
            std::vector<cv::Point2d> points_dist = { shift_point };
            std::vector<cv::Point2d> points_undist(1);
            
            // Корректируем искажения линз
            cv::undistortPoints(points_dist, points_undist, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
            
            // Возвращаем к относительным координатам
            points_undist[0].x -= flow_center_x;
            points_undist[0].y -= flow_center_y;
            
            // Вычисляем поток в радианах с использованием скорректированных точек
            double focal_length_x = camera_matrix_.at<double>(0, 0);
            double focal_length_y = camera_matrix_.at<double>(1, 1);
            
            double flow_x = atan2(points_undist[0].x, focal_length_x);
            double flow_y = atan2(points_undist[0].y, focal_length_y);

            // Инвертируем направление для соответствия системе координат FCU
            float motion_x = flow_y;  // +y means counter-clockwise rotation around Y axis
            float motion_y = -flow_x; // +x means clockwise rotation around X axis

            // Вычисляем время интеграции
            ros::Duration integration_time = msg->header.stamp - prev_stamp_;
            float dt = integration_time.toSec();

            // Публикуем данные для INAV
            std_msgs::Float32MultiArray inav_data;
            inav_data.data.push_back(response * 255);  // quality (0-255)
            inav_data.data.push_back(motion_x);        // motionX в радианах
            inav_data.data.push_back(motion_y);        // motionY в радианах
            inav_flow_pub_.publish(inav_data);

            // Публикуем угловую скорость
            geometry_msgs::TwistStamped velo;
            velo.header.stamp = msg->header.stamp;
            velo.header.frame_id = frame_id_;
            velo.twist.angular.x = motion_x / dt;
            velo.twist.angular.y = motion_y / dt;
            velo_pub_.publish(velo);

            // Обновляем предыдущий кадр
            prev_ = curr_.clone();
            prev_stamp_ = msg->header.stamp;

            // Публикуем отладочное изображение
            if (img_pub_.getNumSubscribers() > 0) {
                // Рисуем вектор потока
                drawFlow(img, shift.x, shift.y, response);
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = msg->header.frame_id;
                out_msg.header.stamp = msg->header.stamp;
                out_msg.encoding = sensor_msgs::image_encodings::MONO8;
                out_msg.image = img;
                img_pub_.publish(out_msg.toImageMsg());
            }
        } catch (const std::exception& e) {
            NODELET_ERROR("Exception in flow callback: %s", e.what());
        }
    }
};

PLUGINLIB_EXPORT_CLASS(OpticalFlow, nodelet::Nodelet) 