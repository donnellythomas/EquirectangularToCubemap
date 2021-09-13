#include <stdio.h>
#include <opencv2/opencv.hpp>
using namespace cv;
// Define our six cube faces. 
// 0 - 3 are side faces, clockwise order
// 4 and 5 are top and bottom, respectively
float faceTransform[6][2] = 
{ 
    {0, 0},
    {M_PI / 2, 0},
    {M_PI, 0},
    {-M_PI / 2, 0},
    {0, -M_PI / 2},
    {0, M_PI / 2}
};

// Map a part of the equirectangular panorama (in) to a cube face
// (face). The ID of the face is given by faceId. The desired
// width and height are given by width and height. 
inline void createCubeMapFace(const Mat &in, Mat &face, 
        int faceId = 0, const int width = -1, 
        const int height = -1) {

    float inWidth = in.cols;
    float inHeight = in.rows;

    // Allocate map
    Mat mapx(height, width, CV_32F);
    Mat mapy(height, width, CV_32F);

    // Calculate adjacent (ak) and opposite (an) of the
    // triangle that is spanned from the sphere center 
    //to our cube face.
    const float an = sin(M_PI / 4);
    const float ak = cos(M_PI / 4);

    const float ftu = faceTransform[faceId][0];
    const float ftv = faceTransform[faceId][1];

    // For each point in the target image, 
    // calculate the corresponding source coordinates. 
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {

            // Map face pixel coordinates to [-1, 1] on plane
            float nx = (float)y / (float)height - 0.5f;
            float ny = (float)x / (float)width - 0.5f;

            nx *= 2;
            ny *= 2;

            // Map [-1, 1] plane coords to [-an, an]
            // thats the coordinates in respect to a unit sphere 
            // that contains our box. 
            nx *= an; 
            ny *= an; 

            float u, v;

            // Project from plane to sphere surface.
            if(ftv == 0) {
                // Center faces
                u = atan2(nx, ak);
                v = atan2(ny * cos(u), ak);
                u += ftu; 
            } else if(ftv > 0) { 
                // Bottom face 
                float d = sqrt(nx * nx + ny * ny);
                v = M_PI / 2 - atan2(d, ak);
                u = atan2(ny, nx);
            } else {
                // Top face
                float d = sqrt(nx * nx + ny * ny);
                v = -M_PI / 2 + atan2(d, ak);
                u = atan2(-ny, nx);
            }

            // Map from angular coordinates to [-1, 1], respectively.
            u = u / (M_PI); 
            v = v / (M_PI / 2);

            // Warp around, if our coordinates are out of bounds. 
            while (v < -1) {
                v += 2;
                u += 1;
            } 
            while (v > 1) {
                v -= 2;
                u += 1;
            } 

            while(u < -1) {
                u += 2;
            }
            while(u > 1) {
                u -= 2;
            }

            // Map from [-1, 1] to in texture space
            u = u / 2.0f + 0.5f;
            v = v / 2.0f + 0.5f;

            u = u * (inWidth - 1);
            v = v * (inHeight - 1);

            // Save the result for this pixel in map
            mapx.at<float>(x, y) = u;
            mapy.at<float>(x, y) = v; 
        }
    }

    // // Recreate output image if it has wrong size or type. 
    // if(face.cols != width || face.rows != height || 
    //     face.type() != in.type()) {
    //     face = Mat(width, height, in.type());
    // }

    // Do actual resampling using OpenCV's remap
    remap(in, face, mapx, mapy, 
         cv::INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
}
Mat importImage(String path){
    Mat image  = imread( path, CV_32F );
    if ( !image.data )
    {
        printf("No image data \n");
    }
    return image;
}

Mat assembleCubemap(Mat in, int cubeSize){
    Mat left, front, right, back, top, bottom;
    
    // Generate each face of the cubemap
    createCubeMapFace(in,front,0, cubeSize,cubeSize);
    createCubeMapFace(in,right,1, cubeSize,cubeSize);
    createCubeMapFace(in,back,2, cubeSize,cubeSize);
    createCubeMapFace(in,left,3, cubeSize,cubeSize);
    createCubeMapFace(in,top,4, cubeSize,cubeSize);
    createCubeMapFace(in,bottom,5, cubeSize,cubeSize);
    
    
    Mat img(cubeSize*3, cubeSize*4, front.type());
    img.setTo(cv::Scalar(.75,.75,0));

    //organize different faces into cubemap
    //can probably fix rotation within createCubeMapFace
    cv::rotate(top, top, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(bottom, bottom, cv::ROTATE_90_COUNTERCLOCKWISE);
    
    top.copyTo(img(Rect(250,0,top.cols, top.rows)));
    left.copyTo(img(Rect(0,250,left.cols, left.rows)));
    front.copyTo(img(Rect(250,250,front.cols, front.rows)));
    right.copyTo(img(Rect(500,250,right.cols, right.rows)));
    back.copyTo(img(Rect(750,250,back.cols, back.rows)));
    bottom.copyTo(img(Rect(250,500,bottom.cols, bottom.rows)));

    return img;
}

int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    Mat in = importImage(argv[1]);
    Mat out = assembleCubemap(in,250);
    imshow("Input", in);
    imshow("Output Image", out);
    waitKey(0);
    return 0;
}
