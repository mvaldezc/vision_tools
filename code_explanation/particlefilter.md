## Particle filter code explanation

The particle filter has a very straight forward usage. As said before, just the following commands are needed:

###### Particle Filter + Color Detection
```
rosrun vision_tools particlefilter
```

One important thing to remark is that the color setpoint for this program can be stablished just drawing a rectangle in the image shown  in the dubugger mode:

![Color setup](https://github.com/marcovc41/vision_tools/blob/master/read_img/pf.PNG)

Then this will appear in the screen:

![Result](https://github.com/marcovc41/vision_tools/blob/master/read_img/ss.png)

#### Code explanation

The libraries, the definitions and the global variables declarations are presented first,

```C++
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <math.h>

#define screen_height 	480
#define screen_width	640

#define N 		5000
#define T			0.2
#define desvV		30
#define desvWpos	30
#define desvWvel	20

int	H=24;
int	S=200;
int	V=180;

cv::Mat frame,frame_HSV;
cv::Mat particle=(cv::Mat_<double>(4,N));
double w[N];
double sum_w=0;
double e[N];
double eH[N],eS[N],eV[N];
double z[N];

int rs[N];
int rsAc[N];
double cumsum[N];
//double pp[4][N];
cv::Mat pp=(cv::Mat_<double>(4,N));;
//int Haccum=0,Hprom=0,Smin=255,Smax=0,Vmin=255,Vmax=0;
int flag_hist=0;
cv::Mat histImage,muestracolor=(cv::Mat_<cv::Vec3b>(50,50) );
cv::Mat A= (cv::Mat_<double>(4,4) << 1,0,T,0, 0,1,0,T, 0,0,1,0, 0,0,0,1);
std::default_random_engine generator;
std::uniform_real_distribution<double> U(0.0,1.0);
std::normal_distribution<double> ND(0.0,1.0);
cv::Point pix;
double x,y,X,Y,XsumW,YsumW;
int xx,yy,flag=0,xinst,yinst;
cv::Rect rectP(0,0,screen_width,screen_height);
cv::Rect recorteR,rect;
cv::Mat recorteM,frame_inst,frame_respaldo;
void weight(void);
void resampling(void);
void propagate(void);
void callBackFunc(int event, int x, int y, int flags, void* userdata);

int main(int argc, char **argv){
	ros::init(argc, argv, "particlefilter");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	// -------------------------- INICIA CODIGO DE OPENCV --------------------------

	cv::VideoCapture cap(0);

	cv::namedWindow("Original Video");
	cv::namedWindow("Histograma");
	cv::namedWindow("Muestra");
	cv::moveWindow("Original Video", 80,60);
	cv::moveWindow("Histograma", 800,60);
	cv::moveWindow("Muestra", 800,300);

	std::cout << "Número de partículas: " << N << std::endl;

	cap >> frame;
	if(frame.empty())  exit(1);

	for(int j=0;j<N;j++){
		particle.at<double>(0,j)=639*U(generator);
		particle.at<double>(1,j)=479*U(generator);
		pix=cv::Point(int(particle.at<double>(0,j)),int(particle.at<double>(1,j)));
		cv::circle(frame, pix, 2, cv::Scalar(0,255,0), -1);
	}

	cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
	cv::imshow("Original Video", frame);;
	cv::waitKey(30);
	weight();
	resampling();

	cv::setMouseCallback("Original Video", callBackFunc);

	while (ros::ok()) {
		if(flag==0){
			cap >> frame;
			if(frame.empty())  exit(1);
			frame.copyTo(frame_respaldo);
			cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
			propagate();
			weight();
			resampling();

		}
		cv::imshow("Original Video", frame);
		if (flag_hist==1){
			cv::imshow("Histograma", histImage);
			cv::imshow("Muestra", muestracolor);
		}
		cv::waitKey(1);
		if(flag==0){
			frame.release();
			frame_HSV.release();
		}

		loop_rate.sleep();
	}    
	// -------------------------- TERMINA CODIGO DE OPENCV --------------------------
	ros::spinOnce();
	return 0;
}


void weight(void){
	sum_w=0;
	XsumW=0;
	YsumW=0;
	for(int k=0;k<N;k++){
		pix=cv::Point(int(particle.at<double>(0,k)),int(particle.at<double>(1,k)));
		cv::Vec3b HSV=frame_HSV.at<cv::Vec3b>(pix.y,pix.x);
		if(H>HSV[0])	eH[k]=H-HSV[0];
		else 		eH[k]=HSV[0]-H;
		if(eH[k]>128)	eH[k]=256-eH[k];

		eS[k]=HSV[1]-S;
		eV[k]=HSV[2]-V;

		e[k]=eH[k]*eH[k]+eS[k]*eS[k]+eV[k]*eV[k];
		w[k]=1/(2.5*desvV)*exp(-(e[k])/(2*desvV*desvV));
		sum_w+=w[k];
		XsumW+=particle.at<double>(0,k)*w[k];
		YsumW+=particle.at<double>(1,k)*w[k];
	}
	X=XsumW/sum_w;
	Y=YsumW/sum_w;
	for(int k=0;k<N;k++){
		w[k]=w[k]/sum_w;
	}
	pix=cv::Point(int(X),int(Y));
	cv::circle(frame, pix, 8, cv::Scalar(255,0,255), -1);
}


void resampling(void){


	z[0]=U(generator);
	cumsum[0]=w[0];
	rs[0]=0;
	if(z[0]<cumsum[0])	rs[0]++;

	for(int k=1;k<N;k++){
		z[k]=U(generator);
		cumsum[k]=cumsum[k-1]+w[k];
		if(z[k]<cumsum[0])	rs[0]++;
	}

	rsAc[0]=rs[0];
	std::sort(std::begin(z),std::end(z));
	for(int i=1;i<N;i++){
		rs[i]=0;
		for(int j=rsAc[i-1];j<N;j++ ){
			if(z[j]<cumsum[i])	rs[i]++;
			else break;
		}
		rsAc[i]=rs[i]+rsAc[i-1];
	}

	for(int j=0;j<rs[0];j++){
			particle.col(0).copyTo(pp.col(j));
	}

	int posicion;
	for(int i=1;i<N;i++){
		for(int j=0;j<rs[i];j++){
			posicion = j+rsAc[i-1];
			particle.col(i).copyTo(pp.col(posicion));
		}
	}

	pp.copyTo(particle);

}


void propagate(void){
	cv::Mat V=(cv::Mat_<double>(4,N));
	for(int j=0;j<N;j++){
		for(int i=0;i<2;i++){
			V.at<double>(i,j)=desvWpos*ND(generator);
		}
		for(int i=2;i<4;i++){
			V.at<double>(i,j)=desvWvel*ND(generator);
		}
	}
	particle=A*particle + V;
	for(int k=0;k<N;k++){
		x=particle.at<double>(0,k);
		y=particle.at<double>(1,k);

		if(x<0) x=0;
		if(y<0) y=0;
		if(x>(screen_width-1)) x=(screen_width-1);
		if(y>(screen_height-1)) y=(screen_height-1);
		particle.at<double>(0,k)=x;
		particle.at<double>(1,k)=y;
		cv::circle(frame, cv::Point(int(x),int(y)), 2, cv::Scalar(0,255,0), -1);
	}
}


void callBackFunc(int event, int x, int y, int flags, void* userdata) {
	if(rectP.contains(cv::Point(x,y))==1){
		switch (event){
			case cv::EVENT_LBUTTONDOWN:
				xx=x;
				yy=y;
				flag=1;
				break;
			case cv::EVENT_MOUSEMOVE:
				if(flag==1){
					flag_hist=0;
					frame.release();
					frame_inst.release();
					frame_respaldo.copyTo(frame_inst);

					xinst=x;
					yinst=y;
					if(xinst>=screen_width)	xinst=screen_width-1;
					if(yinst>=screen_height) yinst=screen_height-1;
					if(xinst<0) xinst=0;
					if(yinst<0) yinst=0;
					if(xinst>xx){
						if(yinst>yy)	rect=cv::Rect(xx,yy,xinst-xx,yinst-yy);
						else 		rect=cv::Rect(xx,yinst,xinst-xx,yy-yinst);
					}
					if(xinst<xx){
						if(yinst>yy)	rect=cv::Rect(xinst,yy,xx-xinst,yinst-yy);
						else 		rect=cv::Rect(xinst,yinst,xx-xinst,yy-yinst);
					}
					cv::rectangle(frame_inst, rect, cv::Scalar(255,0,255), 1);
					recorteR=cv::Rect(rect.x+1,rect.y+1,rect.width-2,rect.height-2);
					frame_inst.copyTo(frame);
				}
				break;
			case cv::EVENT_LBUTTONUP:
				flag=0;
				recorteM.release();
				frame(recorteR).copyTo(recorteM);
				cv::cvtColor(recorteM,recorteM,cv::COLOR_BGR2HSV);
				std::vector<cv::Mat> hsv_planes;
				cv::split(recorteM, hsv_planes);
				cv::Mat h_hist, s_hist, v_hist;
				int histSize = 256;
				float range[] = { 0, 256 } ;
				const float* histRange = { range };
				bool uniform = true; bool accumulate = false;
				cv::Point hmoda,smoda,vmoda;
				cv::calcHist(&hsv_planes[0], 1, 0, cv::Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate );
				cv::calcHist(&hsv_planes[1], 1, 0, cv::Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate );
				cv::calcHist(&hsv_planes[2], 1, 0, cv::Mat(), v_hist, 1, &histSize, &histRange, uniform, accumulate );

				// Draw the histograms for B, G and R
				int hist_w = 512; int hist_h = 400;
				int bin_w = cvRound( (double) hist_w/histSize );

				histImage = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

				/// Normalize the result to [ 0, histImage.rows ]
				cv::normalize(h_hist, h_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
				cv::normalize(s_hist, s_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
				cv::normalize(v_hist, v_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

				/// Draw for each channel
				for( int i = 1; i < histSize; i++ ) {
				   cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(h_hist.at<float>(i-1)) ) ,
							   cv::Point( bin_w*(i), hist_h - cvRound(h_hist.at<float>(i)) ),
							   cv::Scalar( 255, 0, 0), 2, 8, 0  );
				   cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(s_hist.at<float>(i-1)) ) ,
							   cv::Point( bin_w*(i), hist_h - cvRound(s_hist.at<float>(i)) ),
							   cv::Scalar( 0, 255, 0), 2, 8, 0  );
				   cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(v_hist.at<float>(i-1)) ) ,
							   cv::Point( bin_w*(i), hist_h - cvRound(v_hist.at<float>(i)) ),
							   cv::Scalar( 0, 0, 255), 2, 8, 0  );
				}
				cv::minMaxLoc(h_hist,0,0,0,&hmoda);
				cv::minMaxLoc(s_hist,0,0,0,&smoda);
				cv::minMaxLoc(v_hist,0,0,0,&vmoda);
				flag_hist=1;
				std::cout << "Hue :		" << hmoda.y << std::endl;
				std::cout << "Saturation :	" << smoda.y << std::endl;
				std::cout << "Value:		" << vmoda.y << std::endl;
				std::cout << std::endl;
				H=hmoda.y;
				S=smoda.y;
				V=vmoda.y;
				cv::Mat HSVmoda=(cv::Mat_<cv::Vec3b>(1,1) << cv::Vec3b(H,S,V));
				cv::Mat BGRmoda;
				cv::cvtColor(HSVmoda,BGRmoda,cv::COLOR_HSV2BGR);
				cv::rectangle(muestracolor, cv::Rect(0,0,50,50), cv::Scalar(BGRmoda.at<cv::Vec3b>(0,0)[0],BGRmoda.at<cv::Vec3b>(0,0)[1],BGRmoda.at<cv::Vec3b>(0,0)[2]), -1);
				frame_respaldo.copyTo(frame);

				break;
		}
	}
}
```
