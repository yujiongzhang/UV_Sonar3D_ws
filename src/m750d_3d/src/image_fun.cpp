#include "image_fun.h"


//小顶堆
class smacomparison
{
public:
    bool operator()(const uchar &uc1, const uchar &uc2)
    {
        return uc1 > uc2;
    }
};

//大顶堆
class  bigcomparison
{
public:
    bool operator()(const uchar &uc1, const uchar &uc2)
    {
        return uc1 < uc2;
    }
};

//动态亮度增强
void dynamicBrightness(vector<vector<uchar>>& dataMatrix_step2, double index)
{
    uchar L = 255, H = 0;

    //阈值L
    // for (int ping = 3; ping < 52; ping++)
    // {
    //     for (int num = 0; num < 19; num++)
    //     {
    //         L = min(L, dataMatrix[ping][num]);
    //     }
    // }
    L=7;

    //阈值H
    priority_queue<uchar, vector<uchar>, smacomparison> pri_que;

    for (int ping = 0; ping < 294; ping++)
    {
        for (int num = 0; num < 300; num++)
        {
            pri_que.push(dataMatrix_step2[ping][num]);  //小顶堆
            if (pri_que.size() > 100) pri_que.pop(); // 300*294/100=882;
        }
    }
    int Hsum=0;
    for(int i=0; i<100; i++)
    {
        Hsum+=pri_que.top();
        pri_que.pop();
    }
    H = Hsum/100;

    //cout << (int)L << " " << (int)H << endl;

    for (int ping = 0; ping < 294; ping++)
    {
        for (int num = 0; num < 300; num++)
        {
            if(dataMatrix_step2[ping][num]<=L)
            {
                dataMatrix_step2[ping][num]=0;
            }
            else if(dataMatrix_step2[ping][num]>=H)
            {
                dataMatrix_step2[ping][num]=255;
            }
            else
            {
                double ratio = (dataMatrix_step2[ping][num]-L)*1.0/(H-L);
                dataMatrix_step2[ping][num]=255*pow(ratio, index);
            }
        }
    }

    
}

//线性灰度变换
void linearTrans(cv::Mat& src, cv::Mat& dst, double a, double b) {
    int row = src.rows;
    int col = src.cols;

    for(int i=0; i<row; i++) {
        for(int j=0; j<col; j++) {
            int s=a*src.at<uchar>(i,j)+b;
            if(s<0) s=0;
            else if(s>255) s=255;
            dst.at<uchar>(i,j)=s;
        }
    }
}

//幂次灰度变换
void powerTrans(cv::Mat& src, cv::Mat& dst, double para, double exp) {
    // cv::Scalar smean;
    // smean=cv::mean(src);
    // cout<<smean<<endl;
    // src = cv::Mat(dst.size(),dst.type());

    Mat smean, sstd;
    meanStdDev(src, smean, sstd);

    // cout<<smean<<endl;
    // cout<<sstd<<endl;

    double meanval=smean.at<double>(0);

    int row=src.rows;
    int col=src.cols;
    double p=meanval/127.*para;
    // double p=0.01;
    int pixNum=p*row*col;
    // cout<<pixNum<<endl;

    //H
    priority_queue<uchar, vector<uchar>, smacomparison> que_h;
    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            que_h.push(src.at<uchar>(r,c));  //小顶堆
            if (que_h.size() > pixNum) que_h.pop(); 
        }
    }
    int Hsum=0;
    for(int i=0; i<pixNum; i++)
    {
        Hsum+=que_h.top();
        que_h.pop();
    }
    int H = Hsum/pixNum;
    // cout<<Hsum<<"  "<<H<<endl;

    //L
    priority_queue<uchar, vector<uchar>, bigcomparison> que_l;
    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            if(src.at<uchar>(r,c)>0) {
                que_l.push(src.at<uchar>(r,c));  //大顶堆
                // cout<<(int)src.at<uchar>(r,c)<<" ";
            }
            if (que_l.size() > pixNum) que_l.pop(); 
        }
    }
    int Lsum=0;
    for(int i=0; i<pixNum; i++)
    {
        Lsum+=que_l.top();
        que_l.pop();
    }
    int L = Lsum/pixNum+3;
    // int L=10;
    // cout<<Lsum<<"  "<<L<<endl;

    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            int s=src.at<uchar>(r,c);
            if(s<=L)
            {
                dst.at<uchar>(r,c)=0;
            }
            else if(s>=H)
            {
                dst.at<uchar>(r,c)=255;
            }
            else
            {
                double ratio = (s-L)*1.0/(H-L);
                dst.at<uchar>(r,c)=255*pow(ratio, exp);
            }
        }
    }


    
}


uint deltaX[4800];
uint deltaY[4800];
void calDeltaXY()
{
    double Q = -90;
    double dQ;

    int m_nScanLineMax = 4800; //4800条线填充
    int m_nScanPerAngle = 14;  //每一度中有14条线

    dQ = 360. / (double)m_nScanLineMax;
    for (uint i = 0; i < m_nScanLineMax; i++)
    {
        double dx = cos(Q * M_PI / 180.);
        double dy = sin(Q * M_PI / 180.);

        deltaX[i] = (uint32_t)(dx * 65536 + 0.5);
        deltaY[i] = (uint32_t)(dy * 65536 + 0.5);
        Q += dQ;
    }
}

void ringScanForEdge(const cv::Mat& image, cv::Mat& edge, int threshold) 
{
    calDeltaXY();
    for(int i=0; i<4800; i++) {
        uint32_t x = 300 * 65536;
        uint32_t y = 300 * 65536;
        for(int len=0; len<300; len++) {
            x += deltaX[i];
            y += deltaY[i];

            uint row = (uint)0x0000ffff & (y >> 16);
            uint col = (uint)0x0000ffff & (x >> 16);

            if(image.at<uchar>(row, col)>threshold)
            {
                edge.at<uchar>(row, col)=255;
                break;
            }

        }
    }

}


// 环扫边缘点
void ringScanForEdge_Sector(const cv::Mat& image, cv::Mat& edge, int threshold) 
{
    int radius = image.cols;
    calDeltaXY();
    uint row,col;
    for(int i=1300; i<3500; i++) {
        uint32_t x = radius * 65536;
        uint32_t y = 0 * 65536;
        for(int len=0; len< radius; len++) {
            x += deltaX[i];
            y += deltaY[i];

            row = (uint)0x0000ffff & (y >> 16);
            col = (uint)0x0000ffff & (x >> 16);

            if(image.at<uchar>(row, col)>=threshold)
            {
                edge.at<uchar>(row, col)=255;
                break;
            }

        }

    }

}

//从下往上边缘点
void edgeExtraction(const cv::Mat &image, cv::Mat &edge, int threshold)
{

    int width = image.cols;
    int height = image.rows;
    for (int x = 0; x < width; x++)
    {
        for (int y = height-1; y >= 0; y--)//从下往上
        {
            cv::Point2d point_index(x,y);
            if(image.at<uchar>(point_index)>threshold)
            {
                edge.at<uchar>(point_index)=255;
                break;
            }
        }
        
    }
}
