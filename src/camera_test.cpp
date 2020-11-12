#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>



#include <VX/vx.h>
#include <VX/vx_compatibility.h>


#define ERROR_CHECK_STATUS( status ) { \
        vx_status status_ = (status); \
        if(status_ != VX_SUCCESS) { \
            printf("ERROR: failed with status = (%d) at " __FILE__ "#%d\n", status_, __LINE__); \
            exit(1); \
        } \
}

#define ERROR_CHECK_OBJECT( obj ) { \
        vx_status status_ = vxGetStatus((vx_reference)(obj)); \
        if(status_ != VX_SUCCESS) { \
            printf("ERROR: failed with status = (%d) at " __FILE__ "#%d\n", status_, __LINE__); \
            exit(1); \
        } \
}

static void VX_CALLBACK log_callback(vx_context context, vx_reference ref, vx_status status, const vx_char string[])
{
    size_t len = strlen(string);
    if (len > 0) {
        printf("%s", string);
        if (string[len - 1] != '\n')
            printf("\n");
        fflush(stdout);
    }
}

int main(int argc, char** argv)
{   

     int width = 480, height = 360;

    vx_context context = vxCreateContext();
    ERROR_CHECK_OBJECT(context);
    vxRegisterLogCallback(context, log_callback, vx_false_e);
    
    vx_graph graph = vxCreateGraph(context);
    ERROR_CHECK_OBJECT(graph);
    
    vx_image input_rgb_image = vxCreateImage(context, width, height, VX_DF_IMAGE_RGB);
    vx_image output_filtered_image = vxCreateImage(context, width, height, VX_DF_IMAGE_U8);
    ERROR_CHECK_OBJECT(input_rgb_image);
    ERROR_CHECK_OBJECT(output_filtered_image);

    vx_image yuv_image  = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_IYUV);
    vx_image luma_image = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_U8);
    ERROR_CHECK_OBJECT(yuv_image);
    ERROR_CHECK_OBJECT(luma_image);

    vx_threshold hyst = vxCreateThreshold(context, VX_THRESHOLD_TYPE_RANGE, VX_TYPE_UINT8);
    vx_int32 lower = 80, upper = 100;
    vxSetThresholdAttribute(hyst, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_LOWER, &lower, sizeof(lower));
    vxSetThresholdAttribute(hyst, VX_THRESHOLD_ATTRIBUTE_THRESHOLD_UPPER, &upper, sizeof(upper));
    ERROR_CHECK_OBJECT(hyst);
    vx_int32 gradient_size = 3;

    vx_node nodes[] =
    {
        vxColorConvertNode(graph, input_rgb_image, yuv_image),
        vxChannelExtractNode(graph, yuv_image, VX_CHANNEL_Y, luma_image),
        vxCannyEdgeDetectorNode(graph, luma_image, hyst, gradient_size, VX_NORM_L1, output_filtered_image)
    };

    for( vx_size i = 0; i < sizeof( nodes ) / sizeof( nodes[0] ); i++ )
    {
        ERROR_CHECK_OBJECT( nodes[i] );
        ERROR_CHECK_STATUS( vxReleaseNode( &nodes[i] ) );
    }

    ERROR_CHECK_STATUS( vxVerifyGraph( graph ) );
    
   // string option = argv[1];
    cv::Mat input;


    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;

    cv::VideoCapture capture(0); //0 is to read the camera, "video.format" is to read the local video
    if (!capture.isOpened()) {
        ROS_ERROR_STREAM("Failed to open video device\n");
        ros::shutdown();
    }

    //image_transport is responsible for subscription and publishing
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise("camera", 1);

    cv::Mat image;//Mat is the image class defined in OpenCV

    while (ros::ok()) {
        capture >> image; //load

        if (image.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }


            cv::resize(image, image, cv::Size(width, height));
            std::cout<<"Arjun\n";
            
            vx_rectangle_t cv_rgb_image_region;
            cv_rgb_image_region.start_x    = 0;
            cv_rgb_image_region.start_y    = 0;
            cv_rgb_image_region.end_x      = width;
            cv_rgb_image_region.end_y      = height;
            vx_imagepatch_addressing_t cv_rgb_image_layout;
            cv_rgb_image_layout.stride_x   = 3;
            cv_rgb_image_layout.stride_y   = image.step;
            vx_uint8 * cv_rgb_image_buffer = image.data;
            ERROR_CHECK_STATUS( vxCopyImagePatch( input_rgb_image, &cv_rgb_image_region, 0,
                                                &cv_rgb_image_layout, cv_rgb_image_buffer,
                                                VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST ) );
            ERROR_CHECK_STATUS( vxProcessGraph( graph ) );
            vx_rectangle_t rect = { 0, 0, (vx_uint32)width, (vx_uint32)height };
            vx_map_id map_id;
            vx_imagepatch_addressing_t addr;
            void * ptr;
            ERROR_CHECK_STATUS( vxMapImagePatch( output_filtered_image, &rect, 0, &map_id, &addr, &ptr,
                                            VX_READ_ONLY, VX_MEMORY_TYPE_HOST, VX_NOGAP_X ) );
            cv::Mat mat( height, width, CV_8U, ptr, addr.stride_y );
            cv::imshow( "CannyDetect", mat );

        // Convert image from cv::Mat type to sensor_msgs/Image type and publish
        
        /*
                 Cv_bridge can selectively convert color and depth information. In order to use the specified feature encoding, there is a centralized coding form:

                 Mono8: CV_8UC1, grayscale image
                 Mono16: CV_16UC1, 16-bit grayscale image
                 Bgr8: CV_8UC3 with color information and the order of colors is BGR order
                 Rgb8: CV_8UC3 with color information and the order of colors is RGB order
                 Bgra8: CV_8UC4, BGR color image with alpha channel
                 Rgba8: CV_8UC4, CV, RGB color image with alpha channel
        */

        pub_image.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mat).toImageMsg());// Convert image from sensor_msgs/Image type to cv::Mat type
        cv::imshow("camera", image);
        cv::waitKey(3); // opencv refresh image 3ms

        ERROR_CHECK_STATUS( vxUnmapImagePatch( output_filtered_image, map_id ) );

    }
    
       ros::spin();

        ERROR_CHECK_STATUS( vxReleaseGraph( &graph ) );
        ERROR_CHECK_STATUS( vxReleaseImage( &yuv_image ) );
        ERROR_CHECK_STATUS( vxReleaseImage( &luma_image ) );
        ERROR_CHECK_STATUS( vxReleaseImage( &input_rgb_image ) );
        ERROR_CHECK_STATUS( vxReleaseImage( &output_filtered_image ) );
        ERROR_CHECK_STATUS( vxReleaseContext( &context ) );
}