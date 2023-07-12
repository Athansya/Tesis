#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using std::cerr;
using std::cout;
using std::endl;
using std::vector;


int main(int argc, char **argv)
{
	// Características del patrón de calibración
	const cv::Size patternSize(10, 7);
	const float squareSize = 20.0f; // Milímetros

	// Inicializamos cámara con OpenCV
	cv::VideoCapture cap;
	
	// Comando apertura de cámara nvargus
	const char* gst = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=(string)BGRx ! videoconvert ! appsink";

	cap.open(gst);

	// Sanity check
	if (!cap.isOpened())
	{
		cerr << "Fallo en abrir la camara." << endl;
		return -1;
	}

	// Vectores de puntos en el mundo real (3D) y de la imagen (2D)
	vector<vector<cv::Point3f>> worldPoints;
	vector<vector<cv::Point2f>> imagePoints;
	
	// Objetos imagen OpenCV
	cv::Mat frame;
	cv::Mat calibrationFrame;
	cv::Mat gray;	

	// Guardamos frame y mostramos
	while (true)
	{
		cap.read(frame);
		cv::imshow("Cámara", frame);

		// Key
		char key = (char)cv::waitKey(10);

		// Condición de salida
		if ((key == 'q') | (key == 'Q') | (key == 27)) break;

		// Condicion de captura
		else if (key == 'c') {
			
			// Convertimos a grises la imagen
			cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

			// Objeto de bordes del tablero
			vector<cv::Point2f> corners;
			
			// Flag si encontró bordes
			bool found = cv::findChessboardCorners(gray, patternSize, corners);
			if (found) {
			
				// Detección de esquinas
				cv::cornerSubPix(
					gray,
					corners,
					cv::Size(11, 11),
					cv::Size(-1, -1),
					cv::TermCriteria(
						cv::TermCriteria::EPS +
						cv::TermCriteria::MAX_ITER,
						30,
						0.1));

				// Dibujamos las esquinas
				cv::drawChessboardCorners(frame, patternSize, corners, found);
				calibrationFrame = frame.clone();
				cv::imshow("Patrón de calibración detectado", calibrationFrame);

				// Añadimos las coordenadas
				imagePoints.push_back(corners);

				// Creamos un objeto para guardar las esquinas encontradas
				vector<cv::Point3f> objectCorners;

				for (int y = 0; y < patternSize.height; y++) {
					for (int x = 0; x < patternSize.width; x++) {

						objectCorners.push_back(
								cv::Point3f(
									x*squareSize,
									y*squareSize,
									0));
				
					}
				}

				worldPoints.push_back(objectCorners);

				// Imprimimos el número de capturas exitosas
				cout << imagePoints.size() << " Imágenes capturadas." << endl;

				// TODO: Guardamos el frame como imagen
			}
		}
	}
				
	// Calibración
	// TODO: Añadir excepción en caso de no haber tomado ninguna foto.
	cv::Mat cameraMatrix, distCoeffs;
	vector<cv::Mat> rvecs, tvecs;

	cout << "\nCalibración en curso...\n" << endl;
	
	cv::calibrateCamera(
			worldPoints,
			imagePoints,
			frame.size(),
			cameraMatrix,
			distCoeffs,
			rvecs,
			tvecs);

	cout << "\nMatriz de cámara: " << endl << cameraMatrix << endl;
	cout << "\nCoeficientes de distorsión: " << endl << distCoeffs << endl;


	// Limpieza
	cap.release();
	cv::destroyAllWindows();
	
	return 0;
}
