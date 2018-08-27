#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <opencv2\highgui\highgui.hpp>  
#include <iostream>
#include <fstream>
#include<windows.h>

using namespace std;
using namespace cv;

void gotoxy(int x, int y)
{
	COORD c;
	c.X= x; c.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE),c);
}
//////////////////////////////////////////////////////////////////
//函数功能：用向量来做COSα=两向量之积/两向量模的乘积求两条线段夹角
//输入：   线段3个点坐标pt1,pt2,pt0,最后一个参数为公共点
//输出：   线段夹角，单位为角度
//////////////////////////////////////////////////////////////////
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	double angle_line = (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);//余弦值
	return acos(angle_line) * 180 / 3.141592653;
}
//////////////////////////////////////////////////////////////////
//函数功能：采用多边形检测，通过约束条件寻找矩形
//输入：   img 原图像
//          storage 存储
//          minarea，maxarea 检测矩形的最小/最大面积
//          minangle,maxangle 检测矩形边夹角范围，单位为角度
//输出：   矩形序列
//////////////////////////////////////////////////////////////////
void PrintMat(CvMat * A)   //显示矩阵
{
	int i, j;
	//printf("\nMatrix=:");
	for (i = 0; i<A->rows; i++)
	{
		printf("\n");
		switch (CV_MAT_DEPTH(A->type))
		{
		case CV_32F:
		case CV_64F:
			for (j = 0; j<A->cols; j++)
				printf("%9.3f", (float)cvGetReal2D(A, i, j));
			break;
		case CV_8U:
		case CV_16U:
			for (j = 0; j<A->cols; j++)
				printf("%6d", (int)cvGetReal2D(A, i, j));
			break;
		default:
			break;
		}
	}
	printf("\n");
}
////////////////////////////////////////
CvSeq* findSquares4(IplImage* img, CvMemStorage* storage, int minarea, int maxarea, int minangle, int maxangle)
{
	CvSeq* contours;//边缘
	int N = 6;  //阈值分级

	CvSize sz = cvSize(img->width, img->height);

	IplImage* timg = cvCloneImage(img);//拷贝一次img
	IplImage* gray = cvCreateImage(sz, 8, 1); //img灰度图
	IplImage* pyr = cvCreateImage(cvSize(sz.width / 2, sz.height / 2), 8, 3);  //金字塔滤波3通道图像中间变量
	IplImage* tgray = cvCreateImage(sz, 8, 1); ;
	CvSeq* result;
	double s, t;
	CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);

	cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));
	//金字塔滤波 
	cvPyrDown(timg, pyr, 7);
	cvPyrUp(pyr, timg, 7);
	//在3个通道中寻找矩形 
	for (int c = 0; c < 3; c++) //对3个通道分别进行处理 
	{
		cvSetImageCOI(timg, c + 1);
		cvCopy(timg, tgray, 0);  //依次将BGR通道送入tgray         
		for (int l = 0; l < N; l++)
		{
			//不同阈值下二值化
			cvThreshold(tgray, gray, (l + 1) * 255 / N, 255, CV_THRESH_BINARY);

			cvFindContours(gray, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));


			/*for (; contours != 0; contours = contours->h_next)
			{
			printf("***************************************************\n");
			for (int i = 0; i<contours->total; i++)
			{
			CvPoint* p = (CvPoint*)cvGetSeqElem(contours, i);
			printf("p->x=%d,p->y=%d\n", p->x, p->y);


			}

			}*/
			while (contours)
			{ //多边形逼近             
				result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
				//如果是凸四边形并且面积在范围内
				if (result->total == 4 && fabs(cvContourArea(result, CV_WHOLE_SEQ)) > minarea  && fabs(cvContourArea(result, CV_WHOLE_SEQ)) < maxarea &&  cvCheckContourConvexity(result))
				{
					s = 0;
					//判断每一条边
					for (int i = 0; i < 5; i++)
					{
						if (i >= 2)
						{   //角度            
							t = fabs(angle((CvPoint*)cvGetSeqElem(result, i), (CvPoint*)cvGetSeqElem(result, i - 2), (CvPoint*)cvGetSeqElem(result, i - 1)));
							s = s > t ? s : t;
						}
					}
					//这里的S为直角判定条件 单位为角度
					if (s > minangle && s < maxangle)
						for (int i = 0; i < 4; i++)
							cvSeqPush(squares, (CvPoint*)cvGetSeqElem(result, i));

				}
				contours = contours->h_next;

			}

		}
	}
	cvReleaseImage(&gray);
	cvReleaseImage(&pyr);
	cvReleaseImage(&tgray);
	cvReleaseImage(&timg);
	return squares;
}
struct PT
{
	int x;
	int y;
};
struct LINE
{
	PT pStart;
	PT pEnd;
};
Point CrossPoint(const LINE *line1, const LINE *line2)
{
	//    if(!SegmentIntersect(line1->pStart, line1->pEnd, line2->pStart, line2->pEnd))  
	//    {// segments not cross     
	//        return 0;  
	//    }  
	Point pt;
	// line1's cpmponent  
	double X1 = line1->pEnd.x - line1->pStart.x;//b1  
	double Y1 = line1->pEnd.y - line1->pStart.y;//a1  
												// line2's cpmponent  
	double X2 = line2->pEnd.x - line2->pStart.x;//b2  
	double Y2 = line2->pEnd.y - line2->pStart.y;//a2  
												// distance of 1,2  
	double X21 = line2->pStart.x - line1->pStart.x;
	double Y21 = line2->pStart.y - line1->pStart.y;
	// determinant  
	double D = Y1*X2 - Y2*X1;// a1b2-a2b1  
							 //   
	if (D == 0) return 0;
	// cross point  
	pt.x = (X1*X2*Y21 + Y1*X2*line1->pStart.x - Y2*X1*line2->pStart.x) / D;
	// on screen y is down increased !   
	pt.y = -(Y1*Y2*X21 + X1*Y2*line1->pStart.y - X2*Y1*line2->pStart.y) / D;
	// segments intersect.  
	if ((abs(pt.x - line1->pStart.x - X1 / 2) <= abs(X1 / 2)) &&
		(abs(pt.y - line1->pStart.y - Y1 / 2) <= abs(Y1 / 2)) &&
		(abs(pt.x - line2->pStart.x - X2 / 2) <= abs(X2 / 2)) &&
		(abs(pt.y - line2->pStart.y - Y2 / 2) <= abs(Y2 / 2)))
	{
		return pt;
	}
	return 0;
}
//////////////////////////////////////////////////////////////////
//函数功能：画出所有矩形
//输入：   img 原图像
//          squares 矩形序列
//          wndname 窗口名称
//输出：   图像中标记矩形
//////////////////////////////////////////////////////////////////
void drawSquares(IplImage* img, CvSeq* squares, const char* wndname)
{
	int thickness;
	CvSeqReader reader;
	IplImage* cpy = cvCloneImage(img);
	CvPoint pt[4];
	int i;
	cvStartReadSeq(squares, &reader, 0);
	for (i = 0; i < squares->total; i += 4)
	{
		CvPoint* rect = pt;
		int count = 4;
		int d;
		memcpy(pt, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 1, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 2, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		memcpy(pt + 3, reader.ptr, squares->elem_size);
		CV_NEXT_SEQ_ELEM(squares->elem_size, reader);
		//cvPolyLine( cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 3, CV_AA, 0 );
		cvPolyLine(cpy, &rect, &count, 1, 1, CV_RGB(255, 0, 0), 1, CV_AA, 0);//彩色绘制
		if (pt[2].x - pt[3].x > 30) {
			//printf("p0->x=%d,p0->y=%d\n", pt[0].x, pt[0].y);
			//printf("p1->x=%d,p1->y=%d\n", pt[1].x, pt[1].y);
			//printf("p2->x=%d,p2->y=%d\n", pt[2].x, pt[2].y);
			//printf("p3->x=%d,p3->y=%d\n", pt[3].x, pt[3].y);
			LINE line1, line2;
			line1.pStart.x = pt[2].x;
			line1.pStart.y = pt[2].y;
			line1.pEnd.x = pt[3].x;
			line1.pEnd.y = pt[3].y;

			line2.pStart.x = 320;
			line2.pStart.y = 0;
			line2.pEnd.x = 320;
			line2.pEnd.y = 480;

			Point cross = CrossPoint(&line1, &line2);
			//cout << "交点坐标: " << "(" << cross.x << "," << cross.y << ")" << endl;
			d = 240 - cross.y;
			//cout << "距离原点: " << d << endl;
			CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
			CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
			CvMat *rotation = (CvMat*)cvLoad("rotation.xml");
			CvMat *translation = (CvMat*)cvLoad("translation.xml");


			CvMat *pR_matrix = cvCreateMat(3, 3, CV_64FC1);
			//cvInitMatHeader(&pr_vec, 1, 3, CV_64FC1, r_vec, CV_AUTOSTEP);
			//cvInitMatHeader(&pR_matrix, 3, 3, CV_64FC1, &R_matrix, CV_AUTOSTEP);
			cvRodrigues2(rotation, pR_matrix, 0);
			//cvSave("pR_matrix.xml", pR_matrix);
			//PrintMat(pR_matrix);



			double c[12];

			CvMat Ma, Mb, Mc;
			int n;
			//cvInitMatHeader(&Mb, 6, 1, CV_64FC1, b, CV_AUTOSTEP);
			cvInitMatHeader(&Mc, 3, 4, CV_64FC1, c, CV_AUTOSTEP);
			Mat tempM = Mat(&Mc, true);

			//CvMat类转Mat类
			Mat tempMata = Mat(pR_matrix, true);
			Mat tempMatb = Mat(translation, true);
			Mat tempMatbt = tempMatb.t();
			CvMat a = tempMatbt;
			//PrintMat(&a);
			//Mat tempMatc = Mat(&Mc, true);


			n = 0;
			Mat dsttempa = tempM.col(n);   //定义目标矩阵的第n列
			tempMata.col(n).copyTo(dsttempa);	//将行矩阵添加到目标矩阵的第n列

			n = 1;
			Mat dsttempb = tempM.col(n);
			tempMata.col(n).copyTo(dsttempb);

			n = 2;
			Mat dsttempc = tempM.col(n);
			tempMata.col(n).copyTo(dsttempc);

			n = 3;
			Mat dsttempd = tempM.col(n);
			tempMatbt.col(0).copyTo(dsttempd);
			CvMat temp = tempM;   //Mat类转CvMat类

			cvCopy(&temp, &Mc);
			//PrintMat(&Mc);


			double e[] = { 0,0,0,1 };
			double f[16];
			CvMat Me, Mf;
			cvInitMatHeader(&Me, 1, 4, CV_64FC1, e, CV_AUTOSTEP);
			cvInitMatHeader(&Mf, 4, 4, CV_64FC1, f, CV_AUTOSTEP);
			Mat tempMate = Mat(&Me, true);//CvMat类转Mat类
			Mat tempMatf = Mat(&Mc, true);
			Mat tempM1 = Mat(&Mf, true);
			n = 0;
			Mat dsttempe1 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe1);
			n = 1;
			Mat dsttempe2 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe2);
			n = 2;
			Mat dsttempe3 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe3);
			n = 3;
			Mat dsttempe4 = tempM1.row(n);
			tempMate.row(0).copyTo(dsttempe4);
			CvMat temp1 = tempM1;
			cvCopy(&temp1, &Mf);
			//PrintMat(&Mf);//外参矩阵Mf


			double g[12];
			double i[] = { 0,0,0 };
			CvMat  Mg, Mi,Ml;
			

			cvInitMatHeader(&Mi, 3, 1, CV_64FC1, i, CV_AUTOSTEP);
			cvInitMatHeader(&Mg, 3, 4, CV_64FC1, g, CV_AUTOSTEP);
			Mat tempM2 = Mat(&Mg, true);//CvMat类转Mat类
			Mat tempMati = Mat(&Mi, true);
			Mat tempMath = Mat(intrinsic, true);


			n = 0;
			Mat dsttemph1 = tempM2.col(n);   //定义目标矩阵的第n列
			tempMath.col(n).copyTo(dsttemph1);	//将行矩阵添加到目标矩阵的第n列

			n = 1;
			Mat dsttemph2 = tempM2.col(n);
			tempMath.col(n).copyTo(dsttemph2);

			n = 2;
			Mat dsttemph3 = tempM2.col(n);
			tempMath.col(n).copyTo(dsttemph3);

			n = 3;
			Mat dsttemph4 = tempM2.col(n);
			tempMati.col(0).copyTo(dsttemph4);
			CvMat temp2 = tempM2;   //Mat类转CvMat类

			cvCopy(&temp2, &Mg);
			//PrintMat(&Mg);//内参矩阵
			Mat Mf4 = Mat(&Mf, true);
			Mat Mg4 = Mat(&Mg, true);
			Mat Ml2 = Mg4*Mf4;
			CvMat Ml3 = Ml2;
			//PrintMat(&Ml3);

			double picpoint[] = { cross.x,cross.y,1 };
			double origin[] = { 310,246,1 };
			double worldpoint[4];
			double worldpoint2[4];
			CvMat worldpoint1, picpoint1, origin1, worldpoint21;
			cvInitMatHeader(&worldpoint1, 4, 1, CV_64FC1, worldpoint, CV_AUTOSTEP);
			cvInitMatHeader(&worldpoint21, 4, 1, CV_64FC1, worldpoint2, CV_AUTOSTEP);
			cvInitMatHeader(&picpoint1, 3, 1, CV_64FC1, picpoint, CV_AUTOSTEP);
			cvInitMatHeader(&origin1, 3, 1, CV_64FC1, origin, CV_AUTOSTEP);
			Mat worldpointM = Mat(&worldpoint1, true);
			Mat worldpointM2 = Mat(&worldpoint21, true);
			Mat picpointM = Mat(&picpoint1, true);
			Mat originM = Mat(&origin1, true);
			Mat Mg2 = Mat(&Mg, true);//CvMat类转Mat类
			Mat Mginv = Mg2.inv(CV_SVD);
			CvMat Mg3 = Mginv;
			Mat Mf2 = Mat(&Mf, true);//CvMat类转Mat类
			Mat Mfinv = Mf2.inv();
			CvMat Mf3 = Mfinv;
			worldpointM = Mfinv*(Mginv*picpointM) *35;
			worldpointM2 = Mfinv*(Mginv*originM) *35;
			CvMat world2 = worldpointM2;
			//PrintMat(&world);
			//PrintMat(&world2);
			Mat sub = worldpointM - worldpointM2;
			CvMat sub3 = sub;
			//PrintMat(&sub3);
			CvMat sub2 = sub;
			double element1 = CV_MAT_ELEM(sub2, double, 0, 0);
			double element2 = CV_MAT_ELEM(sub2, double, 1, 0);
			double element3 = CV_MAT_ELEM(sub2, double, 2, 0);
			double solution = double cvSqrt(element1*element1 + element2 *element2 + element3*element3);

			//cout << "距离原点: " << solution << "cm" << endl;
            ofstream outfile("data.txt", ios::app);
			streambuf *streams = cout.rdbuf();//保存原来的cout对象
			cout.rdbuf(outfile.rdbuf());
			cout << "距离原点: " << solution << "cm" <<endl;
	
			gotoxy(0,0);
			printf("与圆点位移量=%f cm\n\n", solution);

			
			


			cvReleaseMat(&intrinsic);
			cvReleaseMat(&distortion);
			cvReleaseMat(&rotation);
			cvReleaseMat(&translation);
			//system("pause");

		}
		else
			pt[0].x, pt[0].y;
		//printf("p0->x=%d,p0->y=%d\n", pt[0].x, pt[0].y);
		//printf("p1->x=%d,p1->y=%d\n", pt[1].x, pt[1].y);
		//printf("p2->x=%d,p2->y=%d\n", pt[2].x, pt[2].y);
		//printf("p3->x=%d,p3->y=%d\n", pt[3].x, pt[3].y);
		LINE line1, line2;
		line1.pStart.x = pt[1].x;
		line1.pStart.y = pt[1].y;
		line1.pEnd.x = pt[2].x;
		line1.pEnd.y = pt[2].y;

		line2.pStart.x = 320;
		line2.pStart.y = 0;
		line2.pEnd.x = 320;
		line2.pEnd.y = 640;

		Point cross = CrossPoint(&line1, &line2);
		//cout << "交点坐标: " << "(" << cross.x << "," << cross.y << ")" << endl;
		d = 240 - cross.y;
		//cout << "距离原点: " << d << endl;
		
			CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
			CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
			CvMat *rotation = (CvMat*)cvLoad("rotation.xml");
			CvMat *translation = (CvMat*)cvLoad("translation.xml");

			CvMat *pR_matrix = cvCreateMat(3, 3, CV_64FC1);
			//cvInitMatHeader(&pr_vec, 1, 3, CV_64FC1, r_vec, CV_AUTOSTEP);
			//cvInitMatHeader(&pR_matrix, 3, 3, CV_64FC1, &R_matrix, CV_AUTOSTEP);
			cvRodrigues2(rotation, pR_matrix, 0);
			//cvSave("pR_matrix.xml", pR_matrix);
			//PrintMat(pR_matrix);

			double c[12];
			CvMat Ma, Mb, Mc;
			int n;
			//cvInitMatHeader(&Mb, 6, 1, CV_64FC1, b, CV_AUTOSTEP);
			cvInitMatHeader(&Mc, 3, 4, CV_64FC1, c, CV_AUTOSTEP);
			Mat tempM = Mat(&Mc, true);

			//CvMat类转Mat类
			Mat tempMata = Mat(pR_matrix, true);
			Mat tempMatb = Mat(translation, true);
			Mat tempMatbt = tempMatb.t();
			CvMat a = tempMatbt;
			//PrintMat(&a);
			//Mat tempMatc = Mat(&Mc, true);


			n = 0;
			Mat dsttempa = tempM.col(n);   //定义目标矩阵的第n列
			tempMata.col(n).copyTo(dsttempa);	//将行矩阵添加到目标矩阵的第n列

			n = 1;
			Mat dsttempb = tempM.col(n);
			tempMata.col(n).copyTo(dsttempb);

			n = 2;
			Mat dsttempc = tempM.col(n);
			tempMata.col(n).copyTo(dsttempc);

			n = 3;
			Mat dsttempd = tempM.col(n);
			tempMatbt.col(0).copyTo(dsttempd);
			CvMat temp = tempM;   //Mat类转CvMat类

			cvCopy(&temp, &Mc);
			//PrintMat(&Mc);

			double e[] = { 0,0,0,1 };
			double f[16];
			CvMat Me, Mf;
			cvInitMatHeader(&Me, 1, 4, CV_64FC1, e, CV_AUTOSTEP);
			cvInitMatHeader(&Mf, 4, 4, CV_64FC1, f, CV_AUTOSTEP);
			Mat tempMate = Mat(&Me, true);//CvMat类转Mat类
			Mat tempMatf = Mat(&Mc, true);
			Mat tempM1 = Mat(&Mf, true);
			n = 0;
			Mat dsttempe1 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe1);
			n = 1;
			Mat dsttempe2 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe2);
			n = 2;
			Mat dsttempe3 = tempM1.row(n);
			tempMatf.row(n).copyTo(dsttempe3);
			n = 3;
			Mat dsttempe4 = tempM1.row(n);
			tempMate.row(0).copyTo(dsttempe4);
			CvMat temp1 = tempM1;
			cvCopy(&temp1, &Mf);
			//PrintMat(&Mf);//外参矩阵Mf


			double g[12];
			double i[] = { 0,0,0 };
			CvMat  Mg, Mi;

			cvInitMatHeader(&Mi, 3, 1, CV_64FC1, i, CV_AUTOSTEP);
			cvInitMatHeader(&Mg, 3, 4, CV_64FC1, g, CV_AUTOSTEP);
			Mat tempM2 = Mat(&Mg, true);//CvMat类转Mat类
			Mat tempMati = Mat(&Mi, true);
			Mat tempMath = Mat(intrinsic, true);


			n = 0;
			Mat dsttemph1 = tempM2.col(n);   //定义目标矩阵的第n列
			tempMath.col(n).copyTo(dsttemph1);	//将行矩阵添加到目标矩阵的第n列

			n = 1;
			Mat dsttemph2 = tempM2.col(n);
			tempMath.col(n).copyTo(dsttemph2);

			n = 2;
			Mat dsttemph3 = tempM2.col(n);
			tempMath.col(n).copyTo(dsttemph3);

			n = 3;
			Mat dsttemph4 = tempM2.col(n);
			tempMati.col(0).copyTo(dsttemph4);
			CvMat temp2 = tempM2;   //Mat类转CvMat类

			cvCopy(&temp2, &Mg);
			//PrintMat(&Mg);//内参矩阵
			Mat tempM3;
			tempM3 = tempM2*tempM1;
			CvMat Mk = tempM3;
			//PrintMat(&Mk);
			/////////////////////////////////////////

			double picpoint[] = { cross.x,cross.y,1 };
			double origin[] = { 310,246,1 };
			double worldpoint[4];
			double worldpoint2[4];
			CvMat worldpoint1, picpoint1, origin1, worldpoint21;
			cvInitMatHeader(&worldpoint1, 4, 1, CV_64FC1, worldpoint, CV_AUTOSTEP);
			cvInitMatHeader(&worldpoint21, 4, 1, CV_64FC1, worldpoint2, CV_AUTOSTEP);
			cvInitMatHeader(&picpoint1, 3, 1, CV_64FC1, picpoint, CV_AUTOSTEP);
			cvInitMatHeader(&origin1, 3, 1, CV_64FC1, origin, CV_AUTOSTEP);
			Mat worldpointM = Mat(&worldpoint1, true);
			Mat worldpointM2 = Mat(&worldpoint21, true);
			Mat picpointM = Mat(&picpoint1, true);
			Mat originM = Mat(&origin1, true);
			Mat Mg2 = Mat(&Mg, true);//CvMat类转Mat类
			Mat Mginv = Mg2.inv(CV_SVD);
			CvMat Mg3 = Mginv;
			Mat Mf2 = Mat(&Mf, true);//CvMat类转Mat类
			Mat Mfinv = Mf2.inv();
			CvMat Mf3 = Mfinv;
			worldpointM = Mfinv*(Mginv*picpointM) *35;
			worldpointM2 = Mfinv*(Mginv*originM) * 35;
			CvMat world = worldpointM;
			CvMat world2 = worldpointM2;
			//PrintMat(&world);
			//PrintMat(&world2);
			Mat sub = worldpointM - worldpointM2;
			CvMat sub3 = sub;
			//PrintMat(&sub3);
			CvMat sub2 = sub;
			double element1 = CV_MAT_ELEM(sub2, double, 0, 0);
			double element2 = CV_MAT_ELEM(sub2, double, 1, 0);
			double element3 = CV_MAT_ELEM(sub2, double, 2, 0);
			double solution = double cvSqrt(element1*element1 + element2 *element2 + element3*element3);

			if (cross.y > 246) {
				//cout << "距离原点: " << -solution << "cm" << endl;
				ofstream outfile("data.txt", ios::app);
				streambuf *streams = cout.rdbuf();//保存原来的cout对象
				cout.rdbuf(outfile.rdbuf());
				cout << "距离原点: " << -solution <<"cm" << endl;
				gotoxy(0, 0);
				printf("与圆点位移量=%f cm\n\n", -solution);
			}
			else {
				//cout << "距离原点: " << solution << "cm" << endl;
				ofstream outfile("data.txt", ios::app);
				streambuf *streams = cout.rdbuf();//保存原来的cout对象
				cout.rdbuf(outfile.rdbuf());
				cout << "距离原点: " << solution << "cm" << endl;
				gotoxy(0, 0);
				printf("与圆点位移量=%f cm\n\n", solution);
			}
			cvReleaseMat(&intrinsic);
			cvReleaseMat(&distortion);
			cvReleaseMat(&rotation);
			cvReleaseMat(&translation);
			/////////////////////////////////////////
			//system("pause");

		}
		cvShowImage(wndname, cpy);
		cvReleaseImage(&cpy);
		//cvReleaseImage(&img);
	}



int main()
{
	CvCapture* capture = cvCreateCameraCapture(0);
	IplImage* img0 = 0;
	IplImage* img = cvCreateImage(cvSize(640, 480), 8,3);
	IplImage* img2 = cvCreateImage(cvSize(640, 480), 8, 3);
	IplImage* img3 = cvCreateImage(cvSize(640, 480), 8, 3);
	CvMemStorage* storage = 0;
	int c;
	const char* wndname = "Square Detection Demo"; //窗口名称
	storage = cvCreateMemStorage(0);
	cvNamedWindow(wndname, 1);
    cvWaitKey(1600);
	while (true)
	{

		img0 = cvQueryFrame(capture);

		
		cvSmooth(img0, img, CV_MEDIAN, 3, img0->nChannels);
		cvSmooth(img, img2, CV_GAUSSIAN, 3, img0->nChannels);
		drawSquares(img2, findSquares4(img2, storage, 1500, 3000, 80, 100), wndname);
	    cvClearMemStorage(storage);  //清空存储
		c = cvWaitKey(10);
		if (c == 27)
			break;
	}

	cvReleaseImage(&img0);
	cvClearMemStorage(storage);
	cvDestroyWindow(wndname);
	return 0;
}