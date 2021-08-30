#include "FreeSpace.hpp"

int filter_x[28]={-1,0,1,-3,-2,2,3,-4,4,-4,4,-5,5,-5,5,-5,5,-1,0,1,-3,-2,2,3,-4,4,-4,4};
int filter_y[28]={-5,-5,-5,-4,-4,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,5,5,5,4,4,4,4,3,3,2,2};
int all_x[89]={-1,0,1, 
                -3,-2,-1,0,1,2,3, 
                -4,-3,-2,-1,0,1,2,3,4, 
                -4,-3,-2,-1,0,1,2,3,4, 
                -5,-4,-3,-2,-1,0,1,2,3,4,5,
                -5,-4,-3,-2,-1,0,1,2,3,4,5,
                -5,-4,-3,-2,-1,0,1,2,3,4,5,
                -1,0,1,
                -3,-2-1,0,1,2,3,
                -4,-3,-2,-1,0,1,2,3,4,
                -4,-3,-2,-1,0,1,2,3,4};
int all_y[89]={-5,-5,-5,
                -4,-4,-4,-4,-4,-4,-4,
                -3,-3,-3,-3,-3,-3,-3,-3,-3,
                -2,-2,-2,-2,-2,-2,-2,-2,-2,
                -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
                0,0,0,0,0,0,0,0,0,0,0,
                1,1,1,1,1,1,1,1,1,1,1,
                5,5,5,
                4,4,4,4,4,4,4,
                3,3,3,3,3,3,3,3,3,
                2,2,2,2,2,2,2,2,2};

LivoxFreeSpace::LivoxFreeSpace()
{
    this->pVImg=(unsigned char*)calloc(DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY*DN_SAMPLE_IMG_NZ,sizeof(unsigned char));
}
LivoxFreeSpace::~LivoxFreeSpace()
{
    if(this->pVImg!=NULL)
    {
        free(this->pVImg);
    }
}

void LivoxFreeSpace::FreeSpaceFilter(float* free_space_small, int n , std::vector<float> & free_space)
{
    clock_t t0, t1, t2, t3, t4;
    t0 = clock();
    float pixel_size = 0.2, delta_d_in_r = 0.13, delta_r = 0.15; //delta_d_in_r is smaller than pixel_size and delta_r, to make sure all pixels are covered
    Eigen::MatrixXi src = Eigen::MatrixXi::Zero(100/pixel_size, 100/pixel_size), dst = Eigen::MatrixXi::Zero(100/pixel_size, 100/pixel_size);
    
    std::vector<float> delta_t;
    for (float j = 0.0001; j < 50; j += delta_r) // Prepare the delta theta of different radius
    {
        delta_t.push_back(delta_d_in_r/j);
    } 
    for (int i = 0; i < 360; i++)
    {
        float r = min(free_space_small[i], free_space_small[(i + 1) % n]);
        r = min(r, free_space_small[(i - 1 + n) % n]);
        r = sqrt(r);
        int k = 0;
        for (float j = 0; j < r - 0.5; j += delta_r)
        {
            float dt = delta_t[k++];
            float theta = (i - 180)*FREE_PI/180.0;
            for (float t = theta - 0.01; t < theta + 0.01; t+=dt)
            {
                float x = j*cos(t);
                float y = j*sin(t);
                int m =int((50.0 - x) / pixel_size);
                int nn =int((50.0 - y) / pixel_size);
                src(m, nn) = 1;
            }
        }
    }

    t1 = clock();
    for (int i = 0; i < 360; i++)
    {
        for (float j = 0; j < 49; j += delta_r)
        {
            float x = j * cos((i - 180)*FREE_PI/180.0);
            float y = j * sin((i - 180)*FREE_PI/180.0);
            int m =int((50.0 - x) / pixel_size);
            int nn =int((50.0 - y) / pixel_size);
            int theta = int(atan2f(y, x) * 180.0 / FREE_PI + 180.0 + 0.5);
            theta = theta % n;
            float r = min(free_space_small[theta], free_space_small[(theta + 1) % n]);
            r = min(r, free_space_small[(theta - 1 + n) % n]);
            if (r > j*j + 1)
            {
                int result = 0;
                for (int k = 0; k < 28; k++)  
                {
                    result += src(m + filter_x[k], nn + filter_y[k]);
                }
                if (result < 28) // check if position (m, nn) is in free space
                    break;
                for (int k = 0; k < 89; k++) 
                {
                    dst(m+all_x[k], nn+all_y[k]) = max(1, dst(m+all_x[k], nn+all_y[k]));
                }
                dst(m, nn) = 2;
            }
        }
    }


    t2 = clock();

    for (int i = 0; i < dst.rows(); i++)
    {
        for (int j = 0; j < dst.cols(); j++)
        {
            if (dst(i, j) > 0)
            {
                float x = (100.0 - i*pixel_size) - 50.0;
                float y = (100.0 - j*pixel_size) - 50.0;
                free_space.push_back(x);
                free_space.push_back(y);
                free_space.push_back(255);                    
            }
        }
    }
    t3 = clock();
    // printf("filter time: %f, generate map: %f, conv: %f, fs generate: %f\n\n", 1000.0*(t3 - t0) / CLOCKS_PER_SEC,
    //         1000.0*(t1 - t0) / CLOCKS_PER_SEC, 1000.0*(t2 - t1) / CLOCKS_PER_SEC, 1000.0*(t3 - t2) / CLOCKS_PER_SEC);
}


void LivoxFreeSpace::FreeSpace(float* fPoints, int n, float* free_space, int free_space_n)
{
    int thetaId;
    float distance;

    for(int ii=0; ii < free_space_n; ii++)
    {
        free_space[ii] = 2500;
    }

    for(int pid=0;pid<n;pid++)
    {
        if(fPoints[pid*4+2] < 3) // points of high tree, buildings are rejected
        {
            if (abs(fPoints[pid*4 + 1]) < 1.2 && abs(fPoints[pid*4]) < 2.5) // reject the near points of robot
                continue;
            distance = fPoints[pid*4]*fPoints[pid*4] + fPoints[pid*4+1]*fPoints[pid*4+1];
            thetaId = int((atan2f(fPoints[pid*4+1], fPoints[pid*4]) + FREE_PI) * 180.0 / FREE_PI + 0.5);
            thetaId = thetaId % free_space_n;
            if(free_space[thetaId] > distance && distance > 1)
            {
                free_space[thetaId] = distance;
            }
        }
    }

}

int LivoxFreeSpace::GenerateFreeSpace(float* fPoints1, int pointNum, std::vector<float> & free_space)
{
    clock_t t0, t1, t2, t3, t4;
    t0 = clock();
    // down sampling
    float *fPoints2=(float*)calloc(pointNum*4,sizeof(float));
    int *idtrans1=(int*)calloc(pointNum,sizeof(int));
    int *idtransx=(int*)calloc(pointNum,sizeof(int));
    int *idtransy=(int*)calloc(pointNum,sizeof(int));
    int *idtrans2=(int*)calloc(pointNum,sizeof(int));

    int pntNum = 0;

    this->pVImg=(unsigned char*)calloc(DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY*DN_SAMPLE_IMG_NZ,sizeof(unsigned char));
    std::vector<int> count(DENOISE_IMG_NX*DENOISE_IMG_NY*DENOISE_IMG_NZ, 0);

    for(int pid=0;pid<pointNum;pid++)
    {
        int ix=(fPoints1[pid*4]+DN_SAMPLE_IMG_OFFX)/DENOISE_IMG_DX; 
        int iy=(fPoints1[pid*4+1]+DN_SAMPLE_IMG_OFFY)/DENOISE_IMG_DY; 
        int iz=(fPoints1[pid*4+2]+DN_SAMPLE_IMG_OFFZ)/DENOISE_IMG_DZ;
        idtrans1[pid]=-1;
        if((ix>=0)&&(ix<DENOISE_IMG_NX)&&(iy>=0)&&(iy<DENOISE_IMG_NY)&&(iz>=0)&&(iz<DENOISE_IMG_NZ)) 
        {
            int idx = iz*DENOISE_IMG_NX*DENOISE_IMG_NY+iy*DENOISE_IMG_NX+ix;
            idtrans1[pid]=idx;
            count[idx]++;     
        }
    }

    for(int pid=0;pid<pointNum;pid++)
    {
        if(idtrans1[pid] > -1 && count[idtrans1[pid]] < 3)
        {
            fPoints1[pid*4] = 0;
            fPoints1[pid*4 + 1] = 0;
            fPoints1[pid*4 + 2] = 0;

        }
    }

    for(int pid=0;pid<pointNum;pid++)
    {
        int ix=(fPoints1[pid*4]+DN_SAMPLE_IMG_OFFX)/DN_SAMPLE_IMG_DX;
        int iy=(fPoints1[pid*4+1]+DN_SAMPLE_IMG_OFFY)/DN_SAMPLE_IMG_DY;
        int iz=(fPoints1[pid*4+2]+DN_SAMPLE_IMG_OFFZ)/DN_SAMPLE_IMG_DZ;

        idtrans1[pid]=-1;
        if((ix>=0)&&(ix<DN_SAMPLE_IMG_NX)&&(iy>=0)&&(iy<DN_SAMPLE_IMG_NY)&&(iz>=0)&&(iz<DN_SAMPLE_IMG_NZ))
        {
            idtrans1[pid] = iz*DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY+iy*DN_SAMPLE_IMG_NX+ix;
            idtransx[pid] = ix;
            idtransy[pid] = iy;
            if(pVImg[idtrans1[pid]]==0)//没有访问过，肯定栅格内会有重复的，所以fPoints2只取第一个
            {
                fPoints2[pntNum*4] = fPoints1[pid*4];
                fPoints2[pntNum*4 + 1] = fPoints1[pid*4+1];
                fPoints2[pntNum*4 + 2] = fPoints1[pid*4+2];
                fPoints2[pntNum*4 + 3] = fPoints1[pid*4+3];
                idtrans2[pntNum] = pid;
                pntNum++;
            }
            
            pVImg[idtrans1[pid]] = 1;
        }
    }

    t1 = clock();

    int *pLabelGnd=(int*)calloc(pntNum,sizeof(int));
    int ground_point_num = GroundSegment(pLabelGnd, fPoints2, pntNum, 1.0);

    t2 = clock();

    int agnum = pntNum - ground_point_num;
    float *fPoints3 = (float*)calloc(agnum*4,sizeof(float));
    int agcnt=0;
    for(int ii=0;ii<pntNum;ii++)
    {
        if(pLabelGnd[ii]==0)
        {
            fPoints3[agcnt*4]=fPoints2[ii*4];
            fPoints3[agcnt*4+1]=fPoints2[ii*4+1];
            fPoints3[agcnt*4+2]=fPoints2[ii*4+2];
            fPoints3[agcnt*4+3]=fPoints2[ii*4+3];
            agcnt++;
        }
        
    }
    float *free_space_small = (float*)calloc(360,sizeof(float));
    this->FreeSpace(fPoints3, agnum, free_space_small, 360);
    this->FreeSpaceFilter(free_space_small, 360, free_space);

    free(fPoints2);
    free(idtrans1);
    free(idtrans2);
    free(idtransx);
    free(idtransy);
    free(fPoints3);
    free(pLabelGnd);
    free(this->pVImg);
    free(free_space_small);
    std::vector<int>().swap(count);
    t3 = clock();
    // printf("FreeSpace total Time: %f, Downsample: %f, Ground Segment: %f, FreeSpace: %f\n\n", 1000.0*(t3 - t0) / CLOCKS_PER_SEC, 
    //         1000.0*(t1 - t0) / CLOCKS_PER_SEC, 1000.0*(t2 - t1) / CLOCKS_PER_SEC, 1000.0*(t3 - t2) / CLOCKS_PER_SEC);
}

/*
int LivoxFreeSpace::GroundSegment(int* pLabel,float *fPoints,int pointNum,float fSearchRadius)
Fast ground segmentation using rule-based & plane fitting method 
*/
int LivoxFreeSpace::GroundSegment(int* pLabel,float *fPoints,int pointNum,float fSearchRadius)
{
    int gnum=0;

    float *pGndImg1 = (float*)calloc(GND_IMG_NX1*GND_IMG_NY1,sizeof(float));
    int *tmpLabel1 = (int*)calloc(pointNum,sizeof(int));
    
    for(int ii=0;ii<GND_IMG_NX1*GND_IMG_NY1;ii++)
    {
        pGndImg1[ii]=100;
    }
    for(int pid=0;pid<pointNum;pid++)
    {
        int ix= (fPoints[pid*4]+GND_IMG_OFFX1)/(GND_IMG_DX1+0.000001);
        int iy= (fPoints[pid*4+1]+GND_IMG_OFFY1)/(GND_IMG_DY1+0.000001);
        if(ix<0 || ix>=GND_IMG_NX1 || iy<0 || iy>=GND_IMG_NY1)
        {
            tmpLabel1[pid]=-1;
            continue;
        }

        int iid=ix+iy*GND_IMG_NX1;
        tmpLabel1[pid]=iid;

        if(pGndImg1[iid]>fPoints[pid*4+2])
        {
            pGndImg1[iid]=fPoints[pid*4+2];
        }

    }

    int pnum=0;
    for(int pid=0;pid<pointNum;pid++)
    {
        if(tmpLabel1[pid]>=0)
        {
            if(pGndImg1[tmpLabel1[pid]]+0.4>fPoints[pid*4+2])
            {
                pLabel[pid]=1;
                pnum++;
            }
        }
    }
    free(pGndImg1);
    free(tmpLabel1);


    for(int pid=0;pid<pointNum;pid++)
    {
        if(pLabel[pid]==1)
        {
            if(fPoints[pid*4+2]>1)
            {
                pLabel[pid]=0;
            }
            else if(fPoints[pid*4]*fPoints[pid*4]+fPoints[pid*4+1]*fPoints[pid*4+1]<100)
            {
                if(fPoints[pid*4+2]>0.5)
                {
                    pLabel[pid]=0;
                }
            }

        }
        else
        {
            if(fPoints[pid*4]*fPoints[pid*4]+fPoints[pid*4+1]*fPoints[pid*4+1]<400)
            {
                if(fPoints[pid*4+2]<0.2)
                {
                    pLabel[pid]=1;
                }
            }
        }

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    int in_zone[pointNum] = {0};
    for(int pid=0;pid<pointNum;pid++)
    {
        if(fPoints[pid*4]*fPoints[pid*4]+fPoints[pid*4+1]*fPoints[pid*4+1]<400)
        {
            in_zone[pid] = 1;
            if (pLabel[pid]==1)
            {
                pcl::PointXYZI p;
                p.x = fPoints[pid*4];
                p.y = fPoints[pid*4 + 1];
                p.z = fPoints[pid*4 + 2];
                p.intensity = fPoints[pid*4 + 3];
                cloud->points.push_back(p);
            }
        }
    }

    Eigen::Matrix3f cov;
	Eigen::Vector4f pc_mean;
    Eigen::MatrixXf normal_;

	pcl::computeMeanAndCovarianceMatrix(*cloud, cov, pc_mean);
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
	normal_ = (svd.matrixU().col(2));
	Eigen::Vector3f seeds_mean = pc_mean.head<3>();
	//  normal.T * [x,y,z] = -d
	float d_ = -(normal_.transpose()*seeds_mean)(0,0);
	float th_dist_d_ = 0.3 - d_;
    Eigen::MatrixXf points(pointNum, 3);
    for(int k = 0; k < pointNum; k++)
    {
        points.row(k) << fPoints[k*4], fPoints[k*4+ 1], fPoints[k*4+ 2];
    }

    // ground plane model
    Eigen::VectorXf result = points * normal_;

    for (int k = 0; k < pointNum; k++)
    {
        if (!in_zone[k])
            continue;
        if (result[k] < th_dist_d_)
        {
            pLabel[k] = 1;
        }
        else
        {
            pLabel[k] = 0;
        }
        
    }


    gnum=0;
    for(int pid=0;pid<pointNum;pid++)
    {
        if(pLabel[pid]==1)
        {
            gnum++;
        }
    }

    return gnum;
}

