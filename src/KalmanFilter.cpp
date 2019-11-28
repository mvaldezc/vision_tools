#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/Point.h"

/***********************************VARIABLES GLOBALES***********************************/

// Sistema con aceleración constante
double T=0.1;	
//double T=0.08;	
cv::Mat A= (cv::Mat_<double>(6,6) << 1,0,T,0,0.5*T*T,0, 0,1,0,T,0,0.5*T*T, 0,0,1,0,T,0, 0,0,0,1,0,T, 0,0,0,0,1,0, 0,0,0,0,0,1);
cv::Mat B= (cv::Mat_<double>(6,2) << T*T*T/6,0, 0,T*T*T/6, 0.5*T*T,0, 0,0.5*T*T, T,0, 0,T);
cv::Mat C= (cv::Mat_<double>(2,6) << 1,0,0,0,0,0, 0,1,0,0,0,0);
cv::Mat At,Bt,Ct;

cv::Mat I=(cv::Mat::eye(A.rows,A.cols, CV_64F));

cv::Mat yk;
cv::Mat uk= (cv::Mat_<double>(2,1) << 0,0);
cv::Mat cov;

// Estimación
cv::Mat xk;
cv::Mat Pk;

// Covarianza del ruido
//double desvV=0.1*150, desvW=1.6*150;
double desvV=14, desvW=1000;
cv::Mat R= (cv::Mat_<double>(2,2) << desvV*desvV,0,0,desvV*desvV);
cv::Mat Q= (cv::Mat_<double>(2,2) << desvW*desvW,0,0,desvW*desvW);

// Predicción
cv::Mat Gk,Pk1,xk1;
cv::Mat JZ,JZinv;	// Estas dos variables son únicamente para un paso intermedio de una operación que se dividió

// Video
int flag=1;		// Bandera que indica si hay detección del objeto
double height, width;

geometry_msgs::Point ball_position;

// Banderas
bool dm=0;		// Debugger mode

// Filtrar color
cv::Mat frame, frame_HSV, frame_threshold,frame_threshold1,frame_threshold2, recorte, recorte_threshold;
cv::Mat ajustarcolor, frame_scale;
int bandera, hmin1, hmax1, hmin2, hmax2;
//int H=205, low_S = 80, low_V = 60;
int H=28, low_S = 150, low_V = 140;
int range_H=15, hue_min, hue_max;

void filtroKalman();
cv::Rect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat);
cv::Mat filtrarcolor(cv::Mat imagen);

/************************************** TRACKBARS ***********************************************/

static void H_thresh_trackbar(int, void *)
{
    cv::setTrackbarPos("Hue", "Color Detection", H);
}
static void rangeH_thresh_trackbar(int, void *)
{
    cv::setTrackbarPos("Hue range", "Color Detection", range_H);
}
static void low_S_thresh_trackbar(int, void *)
{
    cv::setTrackbarPos("Low Saturation", "Color Detection", low_S);
}
static void low_V_thresh_trackbar(int, void *)
{
    cv::setTrackbarPos("Low Brightness", "Color Detection", low_V);
}


/*************************************FUNCION PRINCIPAL*************************************/

int main(int argc, char **argv){
    ros::init(argc, argv, "kalmanfilter");
    ros::NodeHandle n;
    ros::Publisher kalman_pub = n.advertise<geometry_msgs::Point>("position",20);
    ros::Rate loop_rate(30);		
    
    /********************************DECLARACION DE VARIABLES********************************/

	// Para detectar blob más grande
	int nccomps,r_x,r_y,r_w,r_h;
	int a=0, amax=0, cntmax=0;
	double cx, cy;
	cv::Mat labels, centroids, stats;
	
	// Para procesar imagen
	cv::Mat prueba, prueba_gray;
	cv::VideoCapture cap;
	
	// Para filtro de Kalman
	cv::transpose(B,Bt);
	Pk=B*Q*Bt;					// Inicialización de matriz de covarianza
	
	/*************************************CODIGO PRINCIPAL*************************************/
	// Selección offline/online
	switch (argc) {
		case 2:	// Si no hay argumentos entonces activa cámara
			std::cout << "Modo online activado" << std::endl;
			cap.open(0);
			cap >> prueba;
			break;
			
		case 3:{	// Si hay dirección de video, úsala
			std::cout << "Modo offline activado" << std::endl;
			cv::Mat imagen;
			cv::VideoCapture capture(argv[2]);
			capture >> prueba;
			capture.release();
			
			cap.open(argv[2]);
			if (!cap.isOpened()){
				std::cout << "Error: no se pudo abrir el archivo" << argv[1] << std::endl;
				return -1;
			}
		}
			break;
			
		default:	
			std::cout << "Error: número erroneo de argumentos" << std::endl;
			exit(1);
	}
	
	

	if(int(*argv[1])-48 == 1){
		dm=1;
		std::cout << "Modo debugger activado" << std::endl;
	}else if(int(*argv[1])-48 == 0){
		std::cout << "Modo debugger desactivado" << std::endl;
	}else{
		std::cout << "Error:Opción no válida" << std::endl;
	}
	
	// Para empezar a la mitad de la pantalla
	cv::cvtColor(prueba, prueba_gray, cv::COLOR_BGR2GRAY);
	height = prueba_gray.rows;
	width = prueba_gray.cols;
	ball_position.x=width/2;
	ball_position.y=height/2;
	std::cout << "Tamaño de la imagen: " << width << " x "<< height <<std::endl;
	
	xk= (cv::Mat_<double>(6,1) << ball_position.x,ball_position.y,0,0,0,0);
	yk= (cv::Mat_<double>(2,1) << ball_position.x,ball_position.y);
	
	if(dm){
		cv::namedWindow("Video");         		
		cv::moveWindow("Video", 200,120);
		cv::namedWindow("Color Detection"); 
		cv::moveWindow("Color Detection", 800,100);

		// Trackbars to set thresholds for HSV values
		cv::createTrackbar("Hue",  			"Color Detection", &H,  		360, H_thresh_trackbar);
		cv::createTrackbar("Hue range", 		"Color Detection", &range_H, 	30,  rangeH_thresh_trackbar);
		cv::createTrackbar("Low Saturation", 	"Color Detection", &low_S, 	150, low_S_thresh_trackbar);
		cv::createTrackbar("Low Brightness", 	"Color Detection", &low_V, 	200, low_V_thresh_trackbar);
	}
	
	
	
	r_x=0;r_y=0;r_w=width;r_h=height;
	cv::Rect rectP(0,0,width,height);
	int cnt=0;
	while(ros::ok()){
			
		/******************************ACONDICIONAMIENTO DE IMAGEN******************************/
		
		cap >> frame;
		if(frame.empty())  exit(1);
		recorte =  frame(rectP);
		recorte_threshold=filtrarcolor(recorte);

		/******************************DETECCION DE BLOB PRINCIPAL******************************/
		
		a=0, amax=0, cntmax=0;
		nccomps = cv::connectedComponentsWithStats (recorte_threshold, labels, stats, centroids);
		
		if(stats.rows>1){
			for(int cntt=1; cntt < stats.rows; cntt++){
				r_w = stats.at<int>(cv::Point(2, cntt));
				r_h = stats.at<int>(cv::Point(3, cntt));
				a = stats.at<int>(cv::Point(4, cntt));
				if(a > amax){
					amax=a;
					cntmax=cntt;
				}
			}
			if(amax>400){
				cx = centroids.at<double>(cv::Point(0, cntmax))+rectP.x;
				cy = centroids.at<double>(cv::Point(1, cntmax))+rectP.y;
				if(dm){
					r_x = stats.at<int>(cv::Point(0, cntmax))+rectP.x;
					r_y = stats.at<int>(cv::Point(1, cntmax))+rectP.y;
					r_w = stats.at<int>(cv::Point(2, cntmax));
					r_h = stats.at<int>(cv::Point(3, cntmax));
					a = stats.at<int>(cv::Point(4, cntmax));
					cv::rectangle(frame, cv::Point(cx-4,cy-4),cv::Point(cx+4,cy+4), cv::Scalar(0,255,0), -1);
					cv::Rect rect(r_x,r_y,r_w,r_h);
					cv::rectangle(frame, rect, cv::Scalar(0,255,0),4);
				}
				flag=1;
				cnt=0;
			}
			else{
				flag=0;
				cnt++;
			}
		}
		else{
			flag=0;
			cnt++;
		}
		
		/***************************************FILTRADO***************************************/
		ball_position.x=xk.at<double>(0,0);
		ball_position.y=xk.at<double>(1,0);
		if(cnt>45||ball_position.x>width*2||ball_position.y>height*2||ball_position.x<-width*2||ball_position.y<-height*2){
			ball_position.x=width/2;
			ball_position.y=height/2;
			ball_position.z=1;
		}	
		
		else 		ball_position.z=0;
				
		cv::Point2f mean(ball_position.x,ball_position.y);
		
		Pk(cv::Rect(0,0,2,2)).copyTo(cov);
		rectP = getErrorEllipse(5.991, mean, cov);
		if(dm){
			cv::rectangle(frame, rectP, cv::Scalar(255,0,255), 2);
			cv::circle(frame, cv::Point(ball_position.x,ball_position.y), 6, cv::Scalar(255,0,0), -1);
		}
		yk=(cv::Mat_<double>(2,1) << cx,cy);
		filtroKalman();
		
		kalman_pub.publish(ball_position);
		ros::spinOnce();
		
		
		/********************************VISUALIZACION DE IMAGEN********************************/
		if(dm){
			if(height==720||height==1080) cv::resize(frame,frame_scale,cv::Size(960,540));
			else cv::resize(frame,frame_scale,cv::Size(480,360));
			ajustarcolor=filtrarcolor(frame_scale);
			cv::imshow("Video", frame_scale);
			cv::imshow("Color Detection", ajustarcolor);
			cv::waitKey(2);
		}
		/**************************PREPARACION PARA SIGUIENTE ITERACION**************************/
		frame.release();
		frame_HSV.release();
		frame_threshold1.release();
		frame_threshold2.release();
		frame_threshold.release();
		recorte.release();
		recorte_threshold.release();
		
		loop_rate.sleep();
	}

    cv::destroyAllWindows();
    	
    // -------------------------- TERMINA CODIGO DE OPENCV --------------------------
    
    return 0;
}


/*************************************FILTRO DE KALMAN*************************************/

void filtroKalman(){

	// Operaciones previas
	cv::transpose(A,At);
	cv::transpose(C,Ct);
	JZ=C*Pk*Ct+R;
	cv::invert(JZ,JZinv);
	
	// Correción
	if(flag==1){
		Gk=Pk*Ct*JZinv;
		xk=xk+Gk*(yk-C*xk);
		Pk=(I - Gk*C)*Pk; }
	
	// Predicción
	xk1=A*xk+B*uk;
	Pk1=A*Pk*At+B*Q*Bt;
	
	Pk=Pk1;
	xk=xk1;
}
    
cv::Rect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat){
	

	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(covmat, eigenvalues, eigenvectors);
	double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));
	double A=chisquare_val*sqrt(eigenvalues.at<double>(0));
	double B=chisquare_val*sqrt(eigenvalues.at<double>(1));
	
	/* En caso de que haya covarianza
	 * Agregar libreria CMath
	double t_xmax = atan2(-B*sin(angle),A*cos(angle));
	double t_ymax = atan2( B*cos(angle),A*sin(angle));
	
	cv::Mat rotmat=(cv::Mat_<double>(2,2) << cos(angle),-sin(angle),sin(angle),cos(angle));
	
	cv::Mat xy_xmax=(cv::Mat_<double>(2,1) << A*cos(t_xmax),B*sin(t_xmax));
	cv::Mat rotxy_xmax=rotmat*xy_xmax;
	cv::Mat xy_ymax=(cv::Mat_<double>(2,1) << A*cos(t_ymax),B*sin(t_ymax));
	cv::Mat rotxy_ymax=rotmat*xy_ymax;

	double xmax=rotxy_xmax.at<double>(0,0);
	double ymax=rotxy_ymax.at<double>(1,0);
	int x=int(mean.x-xmax);
	int y=int(mean.y-ymax);
	int w=int(xmax*2);
	int h=int(ymax*2);*/
	
	int x=int(mean.x-B);
	int y=int(mean.y-A);
	int w=int(B*2);
	int h=int(A*2);
	
	if(x<0){ 
		w+=x;
		x=0;	}
	if(y<0){
		h+=y;
		y=0;	}
	if(x>width) x=width-2;
	if(y>height) y=height-2;
	if(w+x>width) w=width-x-1;
	if(h+y>height) h=height-y-1;
	if(w<1) w=1;
	if(h<1) h=1;

	/*To draw the ellipse
	 * 
	if(angle < 0)
		angle += 6.28318530718;
	angle = 180*angle/3.14159265359;
	cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);*/
	return cv::Rect(x,y,w,h);
	
}

cv::Mat filtrarcolor(cv::Mat imagen){
	cv::cvtColor(imagen, frame_HSV, cv::COLOR_BGR2HSV);
	
	hue_min=(H-range_H)/2;
	hue_max=(H+range_H)/2;
	
	bandera=0;
	
	if(hue_max>180){
		hmin1=0;
		hmax1=hue_max-180;
		hmin2=hue_min;
		hmax2=180;
		bandera=1;	
	}
	if(hue_min<0){
		hmin1=0;
		hmax1=hue_max;
		hmin2=hue_min+180;
		hmax2=180;
		bandera=1;
	}
	if(bandera){
		cv::inRange(frame_HSV, cv::Scalar(hmin1, low_S, low_V), cv::Scalar(hmax1, 255, 255), frame_threshold1);	
		cv::inRange(frame_HSV, cv::Scalar(hmin2, low_S, low_V), cv::Scalar(hmax2, 255, 255), frame_threshold2);
		cv::add(frame_threshold1,frame_threshold2,frame_threshold);
	}
	else{
		cv::inRange(frame_HSV, cv::Scalar(hue_min, low_S, low_V), cv::Scalar(hue_max, 255, 255), frame_threshold);
	}
	return frame_threshold;
}
