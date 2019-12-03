## Kalman filter code explanation

As already mentioned, the way of executing the programs is:

###### Kalman Filter + HAAR/LBP Cascade
```
rosrun vision_tools detect <debugger mode (0/1)> [path to video]
```
###### Kalman Filter + Color Detection
```
rosrun vision_tools kalmanfilter <debugger mode (0/1)> [path to video]
```

Where the debugger mode activated (1) displays in the screen the captured images with all the geometrical drawings to indicate where the soccer ball is found. Deactivating the debugger mode (0) leads to a faster execution and performance. This debugger field is mandatory or otherwise the program will send an error.

By the other hand, the path to a video file is an optional argument, if this is provided, then the algorithm will be tested using this video file. Multiple examples of testing are provided in the `/img` folder of the repository.

Some examples of running the nodes are provided below:

Examples Cascade Classifier:

For offline debugger mode:  `rosrun vision_tools detect 1 '/home/marco/catkin_ws/src/vision_tools/img/prueba1.mp4'`
For realtime debugger mode: `rosrun vision_tools detect 1`

Examples Color Detection:

For offline mode: `rosrun vision_tools kalmanfilter 0`

#### Code explanation

The libraries and global variables declarations are presented first, this includes the matrices of the system model, the kalman filter, the cascade classifier and the miscellaneous variables needed to perform all the operations. This is the part of the code where the KF parameters can be changed.

```C++
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/Point.h"

/********************************************************************************************/
/************************************ VARIABLES GLOBALES ************************************/
/********************************************************************************************/

// Sistema con aceleración constante
double T=0.04;			// tiempo de muestreo

cv::Mat A= (cv::Mat_<double>(6,6) << 1,0,T,0,0.5*T*T,0,  0,1,0,T,0,0.5*T*T,  0,0,1,0,T,0,  0,0,0,1,0,T,  0,0,0,0,1,0,  0,0,0,0,0,1);
cv::Mat B= (cv::Mat_<double>(6,2) << T*T*T/6,0,  0,T*T*T/6,  0.5*T*T,0,  0,0.5*T*T, T,0, 0,T);
cv::Mat C= (cv::Mat_<double>(2,6) << 1,0,0,0,0,0,  0,1,0,0,0,0);
cv::Mat At,Bt,Ct;

cv::Mat xk, Pk ,yk, uk= (cv::Mat_<double>(2,1) << 0,0);

cv::Mat I=(cv::Mat::eye(A.rows,A.cols, CV_64F));
cv::Mat cov;

// Covarianza
double desvV=30, desvW=3000;
cv::Mat R= (cv::Mat_<double>(2,2) << desvV*desvV,0,0,desvV*desvV);
cv::Mat Q= (cv::Mat_<double>(2,2) << desvW*desvW,0,0,desvW*desvW);

// Filtro de Kalman
cv::Mat K,S,Sinv;

// Video
double height, width;
cv::Mat frame;
cv::UMat recorte, recorte_gray;
double FPS_prom=0,FPS=0;
double t0,t1;
int time_cnt=0;
double sum_time=0, prom_time=0;

// Banderas
bool dm=0;		// Debugger mode
int flag=1;		// Bandera que indica si hay detección del objeto

// Detección
geometry_msgs::Point ball_position; //Posicion en pixeles del objeto
cv::Point center;
int posX, posY;
std::vector<cv::Rect> balls;
int cnt=0;		// Para contar frames sin detección seguidos

// Cascada
cv::CascadeClassifier ball_cascade;

// Funciones
void filtroKalman();
cv::Rect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat);
void detect_ball(cv::Rect rectP);
void dibujar_barra(double FPS);

// Medicion del rendimiento
int balls_size=0;
float cntbien=0;
float cntmal=0;
float por_bien=0;
float por_mal=0;
```


In the next section, the main function appears, with the ros node initialization, the program execution rate and the videocapture settings, depending on whether the user provided a video file or not. Finally, the debugger setup occurs.


```C++
/********************************************************************************************/
/************************************** FUNCION PRINCIPAL ***********************************/
/********************************************************************************************/

int main(int argc, char **argv){
	ros::init(argc, argv, "detect");
	ros::NodeHandle n;
	ros::Publisher kalman_pub = n.advertise<geometry_msgs::Point>("position",20);
	ros::Rate loop_rate(30);
	cv::VideoCapture cap;
	//cap.set(CV_CAP_PROP_FPS,30);
	/********************************* I. SELECCIÓN DE MODO *********************************/

	cv::Mat prueba, prueba_gray;

	std::cout << std::endl;
	std::cout << "\033[35m------------------------------------------------------------------------\033[0m" << std::endl;

	// Selección offline/online
	switch (argc) {
		case 2:	// Si no hay argumentos entonces activa cámara
			std::cout << "\033[36mModo de ejecución:\033[33m	real-time\033[0m" << std::endl;
			cap.open(0);	// Abrir la cámara
			if ( !cap.isOpened() ) {
				std::cout << "\033[31mError:\033[0m no se puede conectar a la cámara" << std::endl;
				return -1;
			}
			cap >> prueba;
			break;

		case 3:{	// Si hay dirección de video, úsala
			std::cout << "\033[36mModo de ejecución:\033[33m	offline\033[0m" << std::endl;
			cv::VideoCapture capture(argv[2]);	//Abrir el video
			capture >> prueba;
			capture.release();
			cap.open(argv[2]);
			std::cout << "\033[36mFPS del video:\033[33m		" << cap.get(CV_CAP_PROP_FPS) << "\033[0m" << std::endl;
			if (!cap.isOpened()){
				std::cout << "\033[31mError:\033[0m no se pudo abrir el archivo" << argv[1] << std::endl;
				return -1;
			}
		}
			break;

		default:
			std::cout << "\033[31mError:\033[0m número erroneo de argumentos" << std::endl;
			exit(1);
	}

	// Selección debugger activado/desactivado

	if(int(*argv[1])-48 == 1){		// Un 1 activa el debugger
		dm=1;
		std::cout << "\033[36mModo debugger:\033[33m		activado\033[0m" << std::endl;
	}else if(int(*argv[1])-48 == 0){	// Un 0 desactiva el debugger
		std::cout << "\033[36mModo debugger:\033[33m		desactivado\033[0m" << std::endl;
	}else{
		std::cout << "\033[31mError:\033[0m Opción no válida" << std::endl;
	}
``


Next the debugger mode setup occurs,


```C++

	/*************************** II. INICIALIZACIÓN DE VARIABLES ***************************/
	// Para filtro de Kalman
	cv::transpose(A,At);
	cv::transpose(C,Ct);
	cv::transpose(B,Bt);
	Pk=B*Q*Bt;		// Inicialización de matriz de covarianza

	// Para empezar a la mitad de la pantalla
	cv::cvtColor(prueba, prueba_gray, cv::COLOR_BGR2GRAY);
	height = prueba_gray.rows;
	width = prueba_gray.cols;
	ball_position.x=width/2;
	ball_position.y=height/2;
	center.x=width/2;
	center.y=height/2;
	std::cout << "\033[36mTamaño de la imagen:	\033[33m" << width << " x "<< height << "\033[0m" << std::endl;

	// Para detección y tracking
	cv::Rect rectP(0,0,width,height);	// Para empezar a buscar
	xk= (cv::Mat_<double>(6,1) << ball_position.x,ball_position.y,0,0,0,0);
	yk= (cv::Mat_<double>(2,1) << ball_position.x,ball_position.y);


	// Cargar cascada
	if( !ball_cascade.load("/home/marco/catkin_ws/src/vision_tools/cascade/ballDetector.xml" )) {
        std::cout << "\033[31mError:\033[0m no se detectó la cascada\n" << std::endl;
        return -1;
     }

     // Ventana para debugging

	if(dm){
		//system("gnome-terminal -x sh -c 'htop'");		// Para monitorear el uso de la cpu
		cv::namedWindow("Video");         		
		cv::moveWindow("Video", 200,400);
	}

	// Checar que OpenCL funcione adecuadamente
     std::cout << "\033[36mAceleración GPU:	\033[33m";

     t0 = ros::Time::now().toSec();

	while(ros::ok()){
		/*************************** III. ACONDICIONAMIENTO DE IMAGEN ***************************/
		cap >> frame;
		if(frame.empty())  break;
		frame(rectP).copyTo(recorte);
		cv::cvtColor(recorte, recorte_gray, cv::COLOR_BGR2GRAY );
		cv::equalizeHist(recorte_gray, recorte_gray );

		/******************************* IV. DETECCION DE PELOTA *******************************/

		detect_ball(rectP);

		/************************************ V. FILTRADO ***************************************/


		yk=(cv::Mat_<double>(2,1) << center.x,center.y);

		filtroKalman();

		posX=xk.at<double>(0,0);
		posY=xk.at<double>(1,0);

		// Si el balón se pierde por 45 fotogramas, marcar balón perdido (z=1) y empezar modo de búsqueda
		if(cnt>15||posX>width*2||posY>height*2||posX<-width*2||posY<-height*2){
			ball_position.x=width/2;
			ball_position.y=height/2;
			ball_position.z=1;
		}else{
			ball_position.x=posX;
			ball_position.y=posY;
			ball_position.z=0;
		}

		// Dibujar el rectángulo circunscrito en el elipse de covarianza de posición
		cv::Point2f mean(posX,posY);
		Pk(cv::Rect(0,0,2,2)).copyTo(cov);
		rectP = getErrorEllipse(5.991, mean, cov);

		if(dm){
			cv::rectangle(frame, rectP, cv::Scalar(255,0,255), 2);
			cv::circle(frame, cv::Point(ball_position.x,ball_position.y), 6, cv::Scalar(255,0,0), -1);
		}


		kalman_pub.publish(ball_position);

		ros::spinOnce();


		/********************************VISUALIZACION DE IMAGEN********************************/

		if(dm){	//Mostrar imagen en modo debugger
			cv::imshow("Video", frame);
			cv::waitKey(1);
		}

		/**************************PREPARACION PARA SIGUIENTE ITERACION**************************/
		frame.release();
		recorte.release();
		balls.clear();
		// Obtener frecuencia de muestreo
		t1 = ros::Time::now().toSec();
		FPS=1/(t1-t0);
		time_cnt++;
		if(time_cnt>14){
			std::cout << "\033[0m";
			dibujar_barra(FPS);
			std::cout << " \033[33m	FPS: " << FPS << "\r" << std::flush;
			time_cnt=0;
		}
		sum_time+=(t1-t0);
		t0=t1;
		loop_rate.sleep();
	}

	// Reporte de detección
	por_mal=cntmal/(cntmal+cntbien)*100.0;
	por_bien=cntbien/(cntmal+cntbien)*100.0;
	FPS_prom=(cntmal+cntbien)/sum_time;
	std::cout << "                                                  " << std::endl;
	std::cout << "\033[35m------------------------------------------------------------------------\033[0m" << std::endl;
	std::cout << "\033[1;36mReporte de detección:\033[0m" << std::endl;
	std::cout << "- 1 detección:  " <<  std::fixed << std::setprecision(0) << cntbien << "	\033[36m"
			<<  std::fixed << std::setprecision(2) << por_bien << " %\033[0m" << std::endl;
	std::cout << "- 0 detección:  " <<  std::fixed << std::setprecision(0) << cntmal << "	\033[36m"
			<<  std::fixed << std::setprecision(2) << por_mal << " %\033[0m" << std::endl;
	//std::cout << std::endl;
	std::cout << "- FPS promedio:	" << FPS_prom << std::endl;
	cv::destroyAllWindows();
    	//system("exit");
	// -------------------------- TERMINA CODIGO DE OPENCV --------------------------

	return 0;
}


/*************************************FILTRO DE KALMAN*************************************/

void filtroKalman(){

	// Predicción
	xk=A*xk+B*uk;
	Pk=A*Pk*At+B*Q*Bt;

	if(flag==1){
		// Correción
		S=C*Pk*Ct+R;
		cv::invert(S,Sinv);

		K=Pk*Ct*Sinv;
		xk=xk+K*(yk-C*xk);
		Pk=(I - K*C)*Pk;
	}


}

cv::Rect getErrorEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat){

	cv::Mat eigenvalues, eigenvectors;
	cv::eigen(covmat, eigenvalues, eigenvectors);
	double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));
	double A=chisquare_val*sqrt(eigenvalues.at<double>(0));
	double B=chisquare_val*sqrt(eigenvalues.at<double>(1));

	int x=int(mean.x-B);
	int y=int(mean.y-A);
	int w=int(B*2);
	int h=int(A*2);

	if(x<0){
		//w+=x;
		x=0;	}
	if(y<0){
		//h+=y;
		y=0;	}
	if(x>width) x=width-2;
	if(y>height) y=height-2;
	if(w+x>width) w=width-x-1;
	if(h+y>height) h=height-y-1;
	if(w<1) w=1;
	if(h<1) h=1;

	return cv::Rect(x,y,w,h);
}


void detect_ball(cv::Rect rectP){
	ball_cascade.detectMultiScale( recorte_gray, balls );

	balls_size=balls.size();
	if(balls_size==1)  cntbien++;	// Conteo de frames con deteccion
	else cntmal++;					// Conteo de frames sin deteccion

	if(balls_size>0){		
		center.x=balls[0].x + balls[0].width/2 +rectP.x;
		center.y=balls[0].y + balls[0].height/2 +rectP.y;
		if(dm){
			cv::rectangle(frame, cv::Point(center.x-4,center.y-4),cv::Point(center.x+4,center.y+4), cv::Scalar(0,255,0), -1);
			cv::ellipse( frame, center, cv::Size( balls[0].width/2, balls[0].height/2 ), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4 );
		}
		flag=1;
		cnt=0;
	}
	else{	//Si no detectó pelota sumar uno al contador
		flag=0;
		cnt++;
	}

}

void dibujar_barra(double FPS){
	if(FPS>30)       std::cout << "  [\033[1;31m|||\033[33m||||\033[32m||||||||" << std::flush;
	else if(FPS>=28) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m||||||| " << std::flush;
	else if(FPS>=26) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m||||||  " << std::flush;
	else if(FPS>=24) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m|||||   " << std::flush;
	else if(FPS>=22) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m||||    " << std::flush;
	else if(FPS>=20) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m|||     " << std::flush;
	else if(FPS>=18) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m||      " << std::flush;
	else if(FPS>=16) std::cout << "  [\033[1;31m|||\033[33m||||\033[32m|       " << std::flush;
	else if(FPS>=14) std::cout << "  [\033[1;31m|||\033[33m||||        " << std::flush;
	else if(FPS>=12) std::cout << "  [\033[1;31m|||\033[33m|||         " << std::flush;
	else if(FPS>=10) std::cout << "  [\033[1;31m|||\033[33m||          " << std::flush;
	else if(FPS>=8)  std::cout << "  [\033[1;31m|||\033[33m|          " << std::flush;
	else if(FPS>=6)  std::cout << "  [\033[1;31m|||            " << std::flush;
	else if(FPS>=4)  std::cout << "  [\033[1;31m||             " << std::flush;
	else if(FPS>=2)  std::cout << "  [\033[1;31m|              " << std::flush;
	else 		  std::cout << "  [\033[1;31m               " << std::flush;
	std::cout << "\033[0m] " << std::flush;
}
```
