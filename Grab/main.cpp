///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
using namespace sl;

const std::string helpString = "[d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit";
const std::string prefixPointCloud = "Cloud_"; // Default PointCloud output file prefix
const std::string prefixDepth = "Depth_"; // Default Depth image output file prefix
const std::string path = "./";

int count_save = 0;
int mode_PointCloud = 0;
int mode_Depth = 0;
int PointCloud_format;
int Depth_format;

void printHelp();
std::string PointCloud_format_ext=".ply";
std::string Depth_format_ext=".png";
void setDepthFormatName(int format);
void setPointCloudFormatName(int format);
void savePointCloud(sl::Camera& zed, std::string filename); 
void saveDepth(sl::Camera& zed, std::string filename); 
void saveSbSImage(sl::Camera& zed, std::string filename);
void saveLeftImage(sl::Camera& zed, std::string filename);
void saveLeftImage(cv::Mat& img, std::string filename);
void processKeyEvent(sl::Camera& zed, char &key); 

void myProcessKeyEvent(cv::Mat& img, char &key); 
cv::Mat slMat2cvMat(sl::Mat& input);

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;


    // Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;

    // Open the camera
    ERROR_CODE err = zed.open();
    if (err != ERROR_CODE::SUCCESS) {
        std::cout << "Error " << err << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // Get camera information (ZED serial number)
    auto camera_infos = zed.getCameraInformation();
    printf("Hello! This is my serial number: %d\n", camera_infos.serial_number);

    // Get Camera Information # No use.
    auto distortion = camera_infos.calibration_parameters.left_cam.disto;
    sl::CameraConfiguration conf;
    auto dist = conf.calibration_parameters_raw.left_cam.disto;
    
    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = zed.getCameraInformation().camera_configuration.resolution;
    std::cout << "Resolution : [w, h] = [" << image_size.width << ", " << image_size.height << "]\n";
    // int new_width = image_size.width / 2;
    // int new_height = image_size.height / 2;
    // sl::Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat image_zed(image_size, sl::MAT_TYPE::U8_C3);
    //sl::Mat image_zed_un(image_size, sl::MAT_TYPE::U8_C3);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    //cv::Mat image_ocv_un = slMat2cvMat(image_zed);
    cv::Mat point_cloud;
    
    // We will use [1280, 720] image's center [960, 720];
    cv::Rect myROI(160, 0, 960, 720);
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    //cv::namedWindow("Image_unrectified", cv::WINDOW_AUTOSIZE);
    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, image_size);
            //zed.retrieveImage(image_zed_un, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, image_size);
            image_ocv = slMat2cvMat(image_zed);
            image_ocv = image_ocv(myROI);
            //image_ocv_un = slMat2cvMat(image_zed_un);
            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image", image_ocv);
            //cv::imshow("Image_unrectified", image_ocv_un);

            // Handle key event
            key = cv::waitKey(10);
            myProcessKeyEvent(image_ocv, key);
        }
        else
        {
            std::cout << "Error " << err << ", exit program.\n";
            return EXIT_FAILURE;
        }
        
    }

    cv::destroyAllWindows();
    
    // Close the camera
    zed.close();

    return EXIT_SUCCESS;
}

/**
 * @brief Conversion function between sl::Mat and cv::Mat
 * 
 * @param input 
 * @return * Conversion* 
 */
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

void myProcessKeyEvent(cv::Mat& img, char &key)
{
    switch (key) {
        case 'l': // Save Left image
        case 'L': 
            saveLeftImage(img, std::string("ZED_image") + std::to_string(count_save) + std::string(".png"));
            count_save++;
            break;

        case 'h': // Print help
        case 'H':
            std::cout << helpString << std::endl;
            break;
    }
}

void processKeyEvent(sl::Camera& zed, char &key) 
{
    switch (key) {
        case 'l': // Save Left image
        case 'L': 
        saveLeftImage(zed, std::string("ZED_image") + std::to_string(count_save) + std::string(".png"));
        break;

        case 'd':
        case 'D':
        saveDepth(zed, path + prefixDepth + std::to_string(count_save));
        break;

        case 'n': // Depth format
        case 'N':
        {
            mode_Depth++;
            Depth_format = (mode_Depth % 3);
            setDepthFormatName(Depth_format);
            std::cout << "Depth format: " << Depth_format_ext << std::endl;
        }
        break;

        case 'p':
        case 'P':
        savePointCloud(zed, path + prefixPointCloud + std::to_string(count_save));
        break;


        case 'm': // Point cloud format
        case 'M':
        {
            mode_PointCloud++;
            PointCloud_format = (mode_PointCloud % 4);
            setPointCloudFormatName(PointCloud_format);
            std::cout << "Point Cloud format: " << PointCloud_format_ext << std::endl;
        }
        break;

        case 'h': // Print help
        case 'H':
        std::cout << helpString << std::endl;
        break;

        case 's': // Save side by side image
        case 'S': 
        saveSbSImage(zed, std::string("ZED_image") + std::to_string(count_save) + std::string(".png"));
        break;
    }
    count_save++;
}

void setPointCloudFormatName(int format) 
{
    switch (format) {
        case 0:
        PointCloud_format_ext = ".xyz";
        break;
        case  1:
        PointCloud_format_ext = ".pcd";
        break;
        case  2:
        PointCloud_format_ext = ".ply";
        break;
        case  3:
        PointCloud_format_ext = ".vtk";
        break;
        default:
        break;
    }
}

void setDepthFormatName(int format) {
    switch (format) {
        case  0:
        Depth_format_ext = ".png";
        break;
        case  1:
        Depth_format_ext = ".pfm";
        break;
        case  2:
        Depth_format_ext = ".pgm";
        break;
        default:
        break;
    }
}

void savePointCloud(sl::Camera& zed, std::string filename) 
{
    std::cout << "Saving Point Cloud... " << std::flush;

    sl::Mat point_cloud;
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

    auto state = point_cloud.write((filename + PointCloud_format_ext).c_str());

    if (state == ERROR_CODE::SUCCESS)
        std::cout << "Point Cloud has been saved under " << filename << PointCloud_format_ext << std::endl;
    else
        std::cout << "Failed to save point cloud... Please check that you have permissions to write at this location ("<< filename<<"). Re-run the sample with administrator rights under windows" << std::endl;
}

void saveDepth(sl::Camera& zed, std::string filename) 
{
    std::cout << "Saving Depth Map... " << std::flush;

    sl::Mat depth;
    zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);

    convertUnit(depth, zed.getInitParameters().coordinate_units, UNIT::MILLIMETER);
    auto state = depth.write((filename + Depth_format_ext).c_str());

    if (state == ERROR_CODE::SUCCESS)
        std::cout << "Depth Map has been save under " << filename << Depth_format_ext << std::endl;
    else
		std::cout << "Failed to save depth map... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
}

void saveSbSImage(sl::Camera& zed, std::string filename) 
{
    sl::Mat image_sbs;
    zed.retrieveImage(image_sbs, sl::VIEW::SIDE_BY_SIDE);

    auto state = image_sbs.write(filename.c_str());

	if (state == sl::ERROR_CODE::SUCCESS)
		std::cout << "Side by Side image has been save under " << filename << std::endl;
	else
		std::cout << "Failed to save image... Please check that you have permissions to write at this location (" << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
}

void saveLeftImage(sl::Camera& zed, std::string filename) 
{
    sl::Mat left_image;
    zed.retrieveImage(left_image, sl::VIEW::LEFT_GRAY);

    auto state = left_image.write(filename.c_str());

	if (state == sl::ERROR_CODE::SUCCESS)
		std::cout << "Side by Side image has been save under " << filename << std::endl;
	else
		std::cout << "Failed to save image... Please check that you have permissions to write at this location (" 
        << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
}

void saveLeftImage(cv::Mat& img, std::string filename) 
{
    if(cv::imwrite(filename, img)){
        std::cout << "Side by Side image has been save under " << filename << std::endl;
    }
    else{
        std::cout << "Failed to save image... Please check that you have permissions to write at this location (" 
        << filename << "). Re-run the sample with administrator rights under windows" << std::endl;
    }
}

/**
* This function displays help in console
**/
void printHelp() 
{
    std::cout << " Press 'l' to save Left images\n";
    std::cout << " Press 's' to save Side by side images\n";
    std::cout << " Press 'p' to save Point Cloud\n";
    std::cout << " Press 'd' to save Depth image\n";
    std::cout << " Press 'm' to switch Point Cloud format\n";
    std::cout << " Press 'n' to switch Depth format\n";
}