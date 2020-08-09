#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    if(argc!=3)
    {
        cout<<"usage:feature extraction img1 img2"<<endl;
        return 1;
    }

    // 读取图像
    Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR); // 读取图像保存为Mat形式，默认为灰度图
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    // 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2; //关键点,KeyPoint是OpenCV中存储关键点信息的类，包括pt(坐标)、size(关键点邻域直径大小)、angle(角度，关键点方向)、response(响应强度)
    Mat descriptors_1, descriptors_2;  // 描述子
    /* Ptr智能指针，ORB参数依次为：nfeatures(最多提取的特征点数量)、scaleFactor(尺度参数)、nlevels(高斯金字塔层数)、edgeThreshold(边缘阈值)、firstLevel(第一层索引值)、
    WET_K(用于产生BIREF描述子的点对的个数)、scoreType(用于对特征点进行排序的算法)、patchSize(BIREF描述子的特征点邻域大小) */
    Ptr<ORB> orb = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20); 
                                                                                 
    
    // 第一步：检测Oriented FAST角点位置
    orb->detect(img_1,keypoints_1);
    orb->detect(img_1,keypoints_2);

    //-- 第二步: 根据角点位置计算BRIEF 描述子
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);
    
    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("feature point ORB", outimg1);

    // 第三步：对两幅图像中的描述子进行匹配,使用BRIEF Hamming距离
    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, matches);

    // 第四步匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    // 找出所有匹配之间的最小距离和最大距离,即是最相似的和最不相似的两组点之间的距离
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    cout<<"--Max dist:"<<max_dist<<endl;
    cout<<"--Min dist:"<<min_dist<<endl;

    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误
    // 但有时候最小距离会非常小,设置一个经验值作为下限
    std::vector< DMatch > good_matches;
    for(int i = 0; i < descriptors_1.rows; i++)
    {
        if(matches[i].distance <= max(2*min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    // 第五步: 绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("All match point pairs", img_match);
    imshow("Matching point pairs after optimization", img_goodmatch);
    waitKey(0);

    return 0;

}
