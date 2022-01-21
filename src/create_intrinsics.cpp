#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(void){
    cout << "Create intrinsics for HitVison cam" << endl;

    Mat intrinsics = (Mat_<float>(3, 3) << 764.6849, 0, 645.0494,
                                            0, 764.5846, 501.4334,
                                            0, 0, 1);
    Mat distortions = (Mat_<float>(1, 4) << -0.0307, 0.0465, 0, 0);
    
    FileStorage f_calib;
    f_calib.open("/home/larrydong/eb_record_ws/src/hikrobot_camera/config/calib.yaml", FileStorage::WRITE);
    if(!f_calib.isOpened()){
        cout << "Cannot open f_calib" << endl;
        return -1;
    }
    f_calib << "intrinsics" << intrinsics;
    f_calib << "distortion_coeffs" << distortions;
    f_calib.release();
    cout << "Savd done" << endl;
    return 0;
}