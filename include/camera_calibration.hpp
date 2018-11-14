#ifndef LAR_CALIBRATECAMERA_HPP
#define LAR_CALIBRATECAMERA_HPP

//Forward declared dependencies
class Settings;
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

//Included dependencies
// 

//Classes
class Settings
{
    public:
        Settings() : goodInput(false) {}
        enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
        enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

        void write(cv::FileStorage& fs) const;                        //Write serialization for this class   
        void read(const cv::FileNode& node);                          //Read serialization for this class      
        void validate();       
        cv::Mat nextImage();
       
        static bool readStringList( const std::string& filename, std::vector<std::string>& l );
        static bool isListOfImages( const std::string& filename);
        
    public:
        cv::Size boardSize;              // The size of the board -> Number of items by width and height
        Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
        float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
        int nrFrames;                // The number of frames to use from the input for calibration
        float aspectRatio;           // The aspect ratio
        int delay;                   // In case of a video input
        bool writePoints;            // Write detected feature points
        bool writeExtrinsics;        // Write extrinsic parameters
        bool calibZeroTangentDist;   // Assume zero tangential distortion
        bool calibFixPrincipalPoint; // Fix the principal point at the center
        bool flipVertical;           // Flip the captured images around the horizontal axis
        std::string outputFileName;       // The name of the file where to write
        bool showUndistorsed;        // Show undistorted images after calibration
        std::string input;                // The input ->
        bool useFisheye;             // use fisheye camera model for calibration
        bool fixK1;                  // fix K1 distortion coefficient
        bool fixK2;                  // fix K2 distortion coefficient
        bool fixK3;                  // fix K3 distortion coefficient
        bool fixK4;                  // fix K4 distortion coefficient
        bool fixK5;                  // fix K5 distortion coefficient

        int cameraID;
        std::vector<std::string> imageList;
        size_t atImageList;
        cv::VideoCapture inputCapture;
        InputType inputType;
        bool goodInput;
        int flag;

    private:
        std::string patternToUse;


};

//Function declarations
static inline void read(const cv::FileNode& node, Settings& x, const Settings& default_value);
static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors, bool fisheye);
static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);
static bool runCalibration( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                            std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                            std::vector<float>& reprojErrs,  double& totalAvgErr);
static void saveCameraParams( Settings& s, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                              const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                              const std::vector<float>& reprojErrs, const std::vector<std::vector<cv::Point2f> >& imagePoints,
                              double totalAvgErr );
bool runCalibrationAndSave(Settings& s, cv::Size imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                           std::vector<std::vector<cv::Point2f> > imagePoints);
int calibrateCamera();

#endif