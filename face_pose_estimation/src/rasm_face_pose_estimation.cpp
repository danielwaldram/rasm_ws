//Note that according to the python documentation,
//the Python.h header file should be included first, before any other header file.
//#include <Python.h>
#include <iostream>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);
tf2::Transform adjust_goal_pose(tf2::Transform &ideal_to_goal, float &x_allowable, float &y_allowable, float &z_allowable, float &xrot_allow, float &yrot_allow, float &z_rot_allow);

//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
//The intrinsics are for the camera, so they shouldn't change with the computer
//K is what goes into the camera matrix which is 3x3 and takes into account intrinsic and extrinsic parameters of the camera
double K[9] = {7.3530833553043510e+02, 0.0, 320.0, 0.0, 7.3530833553043510e+02, 240.0, 0.0, 0.0, 1.0};
//D represents the distortion coefficients which represent the radial and tangential distortion that
double D[5] = {-2.3528667558034226e-02, 1.3301431879108856e+00, 0.0, 0.0,
    -6.0786673300480434e+00};

const int debounce_camera_time_delay = 1;
const double centimeter_to_inch_conversion = 1/2.54;

int main(int argc, char **argv){

    double hz = 10.0;

    cv::KalmanFilter KF;    // instantiate Kalman Filter
    int nStates = 18;       // the number of states
    int nMeasurements = 6;  // the number of measured states
    int nInputs = 0;        // then number of action control
    double dt = 1/hz;
    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);  // init Kalman filter
    cv::Mat measurements(nMeasurements, 1, CV_64FC1); measurements.setTo(cv::Scalar(0)); //initializing the measurement vector as zero

    // Code used to send the face pose values to a text file
    std::ofstream myfile;
    myfile.open("goal_pose_data.csv");
    myfile << " time,x_position,y_position,z_position,x_rotation,y_rotation,z_rotation,w_rotation \n";
    //End of code used to send the face pose values to a text file


  ros::init(argc, argv, "rasm_face_pose");
  ros::NodeHandle nh;
  ros::Time start_time = ros::Time::now();
  ros::Time pose_time = ros::Time::now();
  ros::Rate rate(hz);

  // The broadcaster and listener allow recieving info from and sending info to the tf tree
  tf2_ros::TransformBroadcaster tfb;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //These are the two stamped transforms that get sent to the tf tree
  //They should show up in Rviz
  geometry_msgs::TransformStamped stamped_face_to_goal;
  geometry_msgs::TransformStamped stamped_face_to_goal_adj;
  geometry_msgs::TransformStamped stamped_base_to_face;
  geometry_msgs::TransformStamped stamped_base_to_face_filtered;
  // This is the transform between the base and the end effector
  // it is needed so that the transform from the base to the face location can be determined
  geometry_msgs::TransformStamped stamped_base_to_eef;

  // q_eef_to_face is used to define the orientation of the face with respect to the eef.
  //It is also initially set to no rotation and used to initialize the face_to_goal rotation
  //q_base_to_face is the rotation from base to face
  tf2::Quaternion q_eef_to_face;
  q_eef_to_face.setRPY(0, 0, 0);
  tf2::Quaternion q_base_to_face;
  q_base_to_face.setRPY(0,0,0);

  // The transform from base_to_eef and from eef_to_goal are used to calc the transform from base_to_face
  // which is then used to populate the stamped base_to_face transformation.
  tf2::Transform base_to_eef;
  tf2::Transform eef_to_face;
  tf2::Transform base_to_face;
  tf2::Transform base_to_goal;
  tf2::Transform base_to_ideal;
  tf2::Transform face_to_ideal;
  tf2::Transform ideal_to_goal;
  tf2::Transform adj_ideal_to_goal;
  tf2::Transform face_to_goal;

  // The transform between the face and the goal is defined here
  // it does need to be updated in the while loop other than the time stamp
  stamped_face_to_goal.header.frame_id = "face_pose";
  stamped_face_to_goal.child_frame_id = "goal_pose";

  stamped_face_to_goal_adj.header.frame_id = "face_pose";
  stamped_face_to_goal_adj.child_frame_id = "goal_pose_adj";
  // 0.1m (~4") offset in the x direction accounts for the camera being mounted above the screen.
  stamped_face_to_goal.transform.translation.x = 0.02;//0.0508;
  stamped_face_to_goal.transform.translation.y = 0.0;
  // 0.55m offset in the z direction is the viewing distance from users face to screen
  stamped_face_to_goal.transform.translation.z = 0.55;//0.4315;
  stamped_face_to_goal.transform.rotation.x = q_eef_to_face.x();
  stamped_face_to_goal.transform.rotation.y = q_eef_to_face.y();
  stamped_face_to_goal.transform.rotation.z = q_eef_to_face.z();
  stamped_face_to_goal.transform.rotation.w = q_eef_to_face.w();

  // The child and parent IDs are defined here while the transformation values are defined in the loop
  stamped_base_to_face.header.frame_id = "base_link";
  stamped_base_to_face.child_frame_id = "face_pose";

  // The child and parent IDs are defined here while the transformation values are defined in the loop
  stamped_base_to_face_filtered.header.frame_id = "base_link";
  stamped_base_to_face_filtered.child_frame_id = "filtered_face_pose";

  face_to_ideal.setRotation(q_eef_to_face);
  face_to_ideal.setOrigin(tf2::Vector3(0.0, 0.02, 0.55));

  base_to_goal.setRotation(q_eef_to_face);
  base_to_goal.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

  // This vector holds the transforms that are sent to the tf tree
  std::vector<geometry_msgs::TransformStamped> face_and_goal;

  float x_allow = 0.05;
  float y_allow = 0.05;
  float z_allow = 0.05;
  float xrot_allow = 0.1; //about
  float yrot_allow = 0.1;
  float zrot_allow = 0.1;
  // These variables are used for outlier detection
  float max_slope = 0.5;    // This is the max change in position allowable between two frames
  double x_pos_prev = 0.0;  // prev x,y, and z position
  double y_pos_prev = 0.0;
  double z_pos_prev = 0.0;
  bool data_possibly_corrupt = true;

  double x_pos;
  double y_pos;
  double z_pos;

    //Load face detection and pose estimation models (dlib).
    dlib::frontal_face_detector detector;
    dlib::shape_predictor predictor;
    //get_frontal_face_detector gives bounding boxes for ever face in the "image"
    detector = dlib::get_frontal_face_detector();
    // We need a shape_predictor.  This is the tool that will predict face
    // landmark positions given an image and face bounding box.  Here we are just
    // loading the model from the shape_predictor_68_face_landmarks.dat file.
    dlib::deserialize("/home/daniel/catkin_testing_ws/src/face_pose_estimation/data/shape_predictor_68_face_landmarks.dat") >> predictor;
     //fill in cam intrinsics and distortion coefficients
     //CV_64FC1: CV is a prefix, 64 is the number of bits per base matrix element,
     //F is the type of base element. Float, C1 means that there is one channel (greyscale)
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);
    //fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
    //object_pts is an array of object points in the object coordinate system
    //I am not sure how we know where these points are or what the units are
    std::vector<cv::Point3d> object_pts;

    object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
    object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
    object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
    object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
    object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
    object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
    object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
    object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
    object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
    object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
    object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
    object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
    object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
    object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner

    //2D ref points(image coordinates), referenced from detected facial feature
    //image_pts is the array of image points corresponding to the points in object_pts
    std::vector<cv::Point2d> image_pts;

    //result
    //rotation_vec is the output rotation vector that, together with tvec, brings points from the
    //model coorinate system to the camera coordinate system
    cv::Mat rotation_vec;                           //3 x 1
    cv::Mat rotation_mat;                           //3 x 3 R
    //along with the rotation vector, this brings points from the model coordinate system to the
    //camera coordinate system
    cv::Mat translation_vec;                        //3 x 1 T
    cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
    cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

    //reproject 3D points world coordinate axis to verify result pose
    //These points define a 10x10x10 cube which, using the rot and trans vector
    //that tells the position of the face, will get rotated translated translated then projected into the
    //image frame
    std::vector<cv::Point3d> reprojectsrc;
    reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

    //reprojected 2D points
    std::vector<cv::Point2d> reprojectdst;
    reprojectdst.resize(8);

    //temp buf for decomposeProjectionMatrix()
    cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

    //text on screen
    std::ostringstream outtext;
    bool still_roll = false;
    bool still_pitch = false;
    bool still_yaw = false;
    bool still_y = false;
    bool still_x = false;
    time_t time_since_still_roll = 0;
    time_t time_since_still_pitch = 0;
    time_t time_since_still_yaw = 0;
    time_t time_since_still_y = 0;
    time_t time_since_still_x = 0;

    //Determines which camera is connected to. This is specific to a computer, so may change
//Should be 2
    cv::VideoCapture cap(2);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
    if (!cap.isOpened())
        {
        std::cout << "Unable to connect to camera" << std::endl;
        return EXIT_FAILURE;
        }
        //display window
        cv::namedWindow( "demo", cv::WINDOW_NORMAL);
        // camera frame grabbed each loop
        cv::Mat temp;

    //Loop until the escape key is pressed.
    while (1)
    {
      // The transforms to be sent to the tf tree have thier timestamps updated
      pose_time = ros::Time::now();
      stamped_face_to_goal.header.stamp = ros::Time::now();
      stamped_face_to_goal_adj.header.stamp = ros::Time::now();
      stamped_base_to_face.header.stamp = ros::Time::now();
      stamped_base_to_face_filtered.header.stamp = ros::Time::now();
      stamped_base_to_eef = tfBuffer.lookupTransform("base_link", "link_r_6", ros::Time(0));
        //Clear the buffer by grabbing a few frames
        cap >> temp;
        cap >> temp;
        cap >> temp;

        dlib::cv_image<dlib::bgr_pixel> cimg(temp);

        // Detect faces
        std::vector<dlib::rectangle> faces = detector(cimg);

        // Find the pose of each face
        if (faces.size() > 0)
            {
            //std::cout << "DETECTED " << faces.size() << std::endl << std::endl;
            //track features
            dlib::full_object_detection shape = predictor(cimg, faces[0]);

            //draw features
            for (unsigned int i = 0; i < 68; ++i)
                {
                  //temp is the image, the Point is the center of the iamge, 2 is the radius, next is the color and I am not sure about the -1
                cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
                }

            //fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/

            image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
            image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
            image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
            image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
            image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
            image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
            image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
            image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
            image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
            image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
            image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
            image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
            image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
            image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

            //calc pose
            //solvePnP finds an object pse from 3D-2D point correspondences. Function returns the rotation and the translation vectors that transform
            // a 3d point expressed in the object coordinate frame to the camera coordinate frame
            cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);

            //reproject points of the cube defined earlier into the image
            cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

            //draw cube
            cv::line(temp, reprojectdst[0], reprojectdst[1], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[1], reprojectdst[2], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[2], reprojectdst[3], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[3], reprojectdst[0], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[4], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[5], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[6], reprojectdst[7], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[7], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[0], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[1], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[2], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[3], reprojectdst[7], cv::Scalar(0, 0, 255));

            //This line of code finds the point that is located at the center of the face that
            //is being detected. Note: this does not take into account Z
            cv::Point2d my_point = ((reprojectdst[0] + reprojectdst[1] + reprojectdst[2] + reprojectdst[3] + reprojectdst[4] + reprojectdst[5] + reprojectdst[6] + reprojectdst[7])/8);
            //Get the x and y coordinates of the point at the center of the face that is being
            //detected.
            double my_x = my_point.x;
            double my_y = my_point.y;
            //Dividing it by the number of cols or rows makes x/y_for_pose between 0 and 1
            double x_for_pose = my_x/temp.cols;
            double y_for_pose = my_y/temp.rows;
            //Not sure what this is for. Makes x/y_for_pose between .5 and -.5
            x_for_pose = x_for_pose -0.5;
            y_for_pose = y_for_pose -0.5;
            //Again not sure what the purpose of this is
            double x_distance = -translation_vec.at<double>(1,1)*atan(27.6*3.14159265/180)*x_for_pose/0.5;
            double y_distance = translation_vec.at<double>(1,1)*atan(20*3.14159265/180)*y_for_pose/0.5;

            //The rodrigues function takes in a rotation vector and outputs a rotation matrix
            cv::Rodrigues(rotation_vec, rotation_mat);
            // This concatonates the rotation matrix and the translation matrix into a single pose matrix
            cv::hconcat(rotation_mat, translation_vec, pose_mat);
            //I believe this computes the euler angles of the face, although it may be the euler angles of the camera with respect to a world coordinate system
            cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

                          //show angle result
            //outtext << "X: " << std::setprecision(3) << euler_angle.at<double>(0);
            z_pos = -translation_vec.at<double>(1, 1)/100;
            outtext << "Distance from camera [m]: " << std::setprecision(3) << z_pos;
            cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            x_pos = x_distance/100;
            outtext << "x position [m]: " << std::setprecision(3) << x_pos;
            //outtext << "Y: " << std::setprecision(3) << euler_angle.at<double>(1);
            cv::putText(temp, outtext.str(), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "Z: " << std::setprecision(3) << euler_angle.at<double>(2);
            y_pos = y_distance/100;
            outtext << "y position [m]: " << std::setprecision(3) << y_pos;
            cv::putText(temp, outtext.str(), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 1: " << std::setprecision(3) << rotation_vec.at<double>(1, 1);
            double pitch = euler_angle.at<double>(0);
            outtext << "Pitch in degrees: " << std::setprecision(3) << pitch;
            cv::putText(temp, outtext.str(), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 2: " << std::setprecision(3) << rotation_vec.at<double>(2, 1);
            double yaw = euler_angle.at<double>(1);
            outtext << "Yaw in degrees: " << std::setprecision(3) << yaw;
            cv::putText(temp, outtext.str(), cv::Point(50, 120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 3: " << std::setprecision(3) << rotation_vec.at<double>(3, 1);
            double roll = euler_angle.at<double>(2);
            outtext << "Roll in degrees: " << std::setprecision(3) << roll;
            cv::putText(temp, outtext.str(), cv::Point(50, 140), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            char buffer[100];

            outtext << buffer;
            cv::putText(temp, outtext.str(), cv::Point(100, 200), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 255, 255), 3);
            outtext.str("");
            ////////////////// clear image points ////////////////
            image_pts.clear();

            // The stamped transform and transform from the base to the eef is refreshed
            base_to_eef.setRotation(tf2::Quaternion(stamped_base_to_eef.transform.rotation.x,stamped_base_to_eef.transform.rotation.y, stamped_base_to_eef.transform.rotation.z, stamped_base_to_eef.transform.rotation.w));
            base_to_eef.setOrigin(tf2::Vector3(stamped_base_to_eef.transform.translation.x, stamped_base_to_eef.transform.translation.y, stamped_base_to_eef.transform.translation.z));

            // The eef_to_face transform is updated based on the values calculated using the face pose detector
            q_eef_to_face.setRPY(-yaw*M_PI/180, -pitch*M_PI/180, -roll*M_PI/180);
            eef_to_face.setRotation(q_eef_to_face);
            eef_to_face.setOrigin(tf2::Vector3(y_pos, -x_pos, -z_pos));

            // The transform from the base to the face is updated
            base_to_face = base_to_eef*eef_to_face;

            base_to_ideal = base_to_face*face_to_ideal;

            ideal_to_goal = base_to_ideal.inverse()*base_to_goal;

            adj_ideal_to_goal = adjust_goal_pose(ideal_to_goal, x_allow, y_allow, z_allow, xrot_allow, yrot_allow, zrot_allow);
            // updating the face to goal
            face_to_goal = face_to_ideal*adj_ideal_to_goal;
            base_to_goal = base_to_face*face_to_goal;

            q_base_to_face = face_to_goal.getRotation();
            stamped_face_to_goal_adj.transform.rotation.x = q_base_to_face.x();
            stamped_face_to_goal_adj.transform.rotation.y = q_base_to_face.y();
            stamped_face_to_goal_adj.transform.rotation.z = q_base_to_face.z();
            stamped_face_to_goal_adj.transform.rotation.w = q_base_to_face.w();
            stamped_face_to_goal_adj.transform.translation.x = face_to_goal.getOrigin()[0];
            stamped_face_to_goal_adj.transform.translation.y = face_to_goal.getOrigin()[1];
            stamped_face_to_goal_adj.transform.translation.z = face_to_goal.getOrigin()[2];


            q_base_to_face = base_to_face.getRotation();
            stamped_base_to_face.transform.rotation.x = q_base_to_face.x();
            stamped_base_to_face.transform.rotation.y = q_base_to_face.y();
            stamped_base_to_face.transform.rotation.z = q_base_to_face.z();
            stamped_base_to_face.transform.rotation.w = q_base_to_face.w();

            stamped_base_to_face.transform.translation.x = base_to_face.getOrigin()[0];
            stamped_base_to_face.transform.translation.y = base_to_face.getOrigin()[1];
            stamped_base_to_face.transform.translation.z = base_to_face.getOrigin()[2];

            measurements.at<double>(0) = y_pos;             // x
            measurements.at<double>(1) = -x_pos;            // y
            measurements.at<double>(2) = -z_pos;            // z
            measurements.at<double>(3) = -yaw*M_PI/180;     // yaw
            measurements.at<double>(4) = -pitch*M_PI/180;   // pitch
            measurements.at<double>(5) = -roll*M_PI/180;    // roll




            myfile << (ros::Time::now() - start_time) << "," <<  stamped_base_to_face.transform.translation.x;
            myfile << "," <<  stamped_base_to_face.transform.translation.y << "," <<  stamped_base_to_face.transform.translation.z;
            myfile << "," <<  stamped_base_to_face.transform.rotation.x << "," <<  stamped_base_to_face.transform.rotation.y;
            myfile << "," <<  stamped_base_to_face.transform.rotation.z << "," <<  stamped_base_to_face.transform.rotation.w;
            myfile << "\n";

            rate.sleep();
        }

        KF.predict();   // First predict, to update the internal statePre variable
        cv::Mat estimated = KF.correct(measurements);
        if (faces.size() > 0){
            if(data_possibly_corrupt == true){
                // set the data possibly corrupt as false for next loop and set the previous positions to be the current positions
                data_possibly_corrupt = false;
                x_pos_prev = x_pos;
                y_pos_prev = y_pos;
                z_pos_prev = z_pos;
                // Send the unfiltered/unadjusted goal
                face_and_goal.clear();
                face_and_goal.push_back(stamped_base_to_face);
                face_and_goal.push_back(stamped_face_to_goal);
                tfb.sendTransform(face_and_goal);
            }
            else if(pow(pow((x_pos - x_pos_prev),2)+ pow((y_pos - y_pos_prev),2) + pow((z_pos-z_pos_prev), 2),0.5) > max_slope){
                // Send the unfiltered/unadjusted goal
                face_and_goal.clear();
                face_and_goal.push_back(stamped_base_to_face);
                face_and_goal.push_back(stamped_face_to_goal);
                tfb.sendTransform(face_and_goal);
                // Set data as possibly corrupt to true and don't hold on to this data as previous
                data_possibly_corrupt = true;
            }else{
                // The eef_to_face transform is updated based on the values calculated using the face pose detector
                q_eef_to_face.setRPY(estimated.at<double>(9),estimated.at<double>(10), estimated.at<double>(11));
                eef_to_face.setRotation(q_eef_to_face);
                eef_to_face.setOrigin(tf2::Vector3(estimated.at<double>(0),estimated.at<double>(1), estimated.at<double>(2)));

                // The transform from the base to the face is updated
                base_to_face = base_to_eef*eef_to_face;

                q_base_to_face = base_to_face.getRotation();
                stamped_base_to_face_filtered.transform.rotation.x = q_base_to_face.x();
                stamped_base_to_face_filtered.transform.rotation.y = q_base_to_face.y();
                stamped_base_to_face_filtered.transform.rotation.z = q_base_to_face.z();
                stamped_base_to_face_filtered.transform.rotation.w = q_base_to_face.w();

                stamped_base_to_face_filtered.transform.translation.x = base_to_face.getOrigin()[0];
                stamped_base_to_face_filtered.transform.translation.y = base_to_face.getOrigin()[1];
                stamped_base_to_face_filtered.transform.translation.z = base_to_face.getOrigin()[2];


                face_and_goal.clear();
                face_and_goal.push_back(stamped_base_to_face);
                //face_and_goal.push_back(stamped_base_to_face_filtered);
                face_and_goal.push_back(stamped_face_to_goal);
                face_and_goal.push_back(stamped_face_to_goal_adj);
                tfb.sendTransform(face_and_goal);
                // Set the previous face pose to be the current for the next loop
                x_pos_prev = x_pos;
                y_pos_prev = y_pos;
                z_pos_prev = z_pos;
            }

        }
 //press esc to end
        cv::imshow("demo", temp);
        unsigned char key = cv::waitKey(1);
        if (key == 27)
            {
            myfile.close();
            break;
            }
          //r.sleep();
    }
    return 0;
}


void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
                 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
       /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}

// I need to determine what roll, pitch, and yaw correspond to in terms of x, y, and z
tf2::Transform adjust_goal_pose(tf2::Transform &ideal_to_goal, float &x_allowable, float &y_allowable, float &z_allowable, float &roll_allowable, float &pitch_allowable, float &yaw_allowable){
    tf2::Transform adj_ideal_to_goal;
    tf2::Quaternion quat;

    quat = ideal_to_goal.getRotation();     // getting the rotation from the ideal to the goal pose in terms of a quaternion
    tf2::Matrix3x3 m(quat);                     // expressing the rotation in terms of a 3x3 matrix
    double roll, pitch, yaw;                // getting the quaternion in terms of roll pitch and yaw for comparision to the threshold
    m.getRPY(yaw, pitch, roll);             // expressing roll, pitch, and yaw in terms of how they relate to the joints of the RASM

    // Adjusting the transform based on whether the threshold is exceeded for all 6-DOF
    double x_adjusted, y_adjusted, z_adjusted, roll_adjusted, pitch_adjusted, yaw_adjusted;
    if(abs(ideal_to_goal.getOrigin()[0]) > x_allowable){
        x_adjusted = 0;
    }else{
        x_adjusted = ideal_to_goal.getOrigin()[0];
    }
    if(abs(ideal_to_goal.getOrigin()[1]) > y_allowable){
        y_adjusted = 0;
    }else{
        y_adjusted = ideal_to_goal.getOrigin()[1];
    }
    if(abs(ideal_to_goal.getOrigin()[2]) > z_allowable){
        z_adjusted = 0;
    }else{
        z_adjusted = ideal_to_goal.getOrigin()[2];
    }
    if(abs(roll) > roll_allowable){
        roll_adjusted = 0;
    }else{
        roll_adjusted = roll;
    }if(abs(pitch) > pitch_allowable){
        pitch_adjusted = 0;
    }else{
        pitch_adjusted = pitch;
    }if(abs(yaw) > yaw_allowable){
        yaw_adjusted = 0;
    }else{
        yaw_adjusted = yaw;
    }

    //Will need to do for every case but will start testing with just this one
    adj_ideal_to_goal.setOrigin(tf2::Vector3(x_adjusted, y_adjusted, z_adjusted));
    quat.setRPY(yaw_adjusted, pitch_adjusted, roll_adjusted);
    adj_ideal_to_goal.setRotation(quat);
    return adj_ideal_to_goal;
}

