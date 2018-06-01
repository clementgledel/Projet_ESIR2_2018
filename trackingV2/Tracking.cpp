#include "Tracking.h"

Tracking::Tracking(string videoName) : m_cap(videoName), cpt_reinit_histo(0)
{
	// On récupère la première image de la frame
	m_cap >> m_current_frame;
}

Tracking::~Tracking()
{
	destroyAllWindows();
	m_cap.release();
}

/** 
 *  Méthode permettant d'initialiser la cible sur la vidéo ainsi que son histogramme associé
 *  width : largeur de la fenetre de tracking
 *  height : hauteur de la fenetre de tracking
 *  channels, histSize, range et ranges : parmètres de création de l'histogramme
**/
void Tracking::initializeHistogram(int width, int height, const int channels[], const int histSize[], float range[], const float *ranges[])
{
	// Recherche du centre d'un oiseau sur l'image
	Point2f target = searchTarget();
	// Si centre trouvé alors
	if (target.x != -1)
	{
		// Initialisation de la nouvelle fenetre de trracking avec le point rédupéré
		m_track_window = Rect(target.x - width / 2, target.y - height / 2, width, height) & Rect(0, 0, m_current_frame.cols, m_current_frame.rows);

		// Récupération de l'image qui a permit de trouver le centre de l'oiseau sur la vidéo
		Mat pic = m_geo.getPicture();
		// et initialisation de l'image située dans la fenetre
		Mat roi = pic(m_track_window);

		// Calcul roi histogram
		calculHistogram(roi, channels, histSize, range, ranges, true);

		// Initialisation du filtre de Kalman avec la tracking window 
		initKalman();

		// Cible défini
		m_isLost = false;
	}
	// Sinon cible perdue
	else
		m_isLost = true;
}

/**
*  Méthode permettant de trouver une cible dans la vidéo
*  return le point du centre de la cible
**/
Point2f Tracking::searchTarget()
{
	// Initalisation du point à retourner en dehors de la zone d'acceptation du point
	Point2f oiseau = Point2f(m_current_frame.cols - 1, m_current_frame.rows - 1);
	// Compteur d'itération dans la boucle
	int cpt = 0;
	// Si point non-trouvé dans la partie gauche de l'image
	while (oiseau.x > m_current_frame.cols / 2)
	{
		// Récupération de la frame suivante
		m_current_frame = Mat();
		m_cap >> m_current_frame;

		// Si la frame est vide ou si après 10 itération nous n'avons pas trouvé de cible alors on renvoie un point en (-1,-1)
		if (m_current_frame.empty() || cpt > 10)
			return Point2f(-1, -1);

		// Ajout de l'image actuel dans la classe de recherche de point
		m_geo.setPictures(m_current_frame);

		// Appel de la méthode qui compute la recherche de point après avoir inscrit au minimum 3 images dans la classe
		m_geo.DO();

		// Récupération du point du centre de l'oiseau (retourne un point en dehors de l'image si aucun point)
		oiseau = m_geo.getOiseau();

		imshow("image", m_current_frame);
		cpt++;
	}
	return oiseau;
}

/**
 *  Méthode permettant de calculer l'histogramme dans une image donnée
 *  roi : image dans lequel on calcul l'histogramme
 *  channels, histSize, range et ranges : parmètres de création de l'histogramme
 *  usemask : booléen permettant de demander un masque dans la création de l'histogramme
 *  (masque entre Scalar(140., 0., 0.), Scalar(255., 255., 150.) HSV seulement)
**/
void Tracking::calculHistogram(Mat &roi, const int channels[], const int histSize[], float range[], const float *ranges[], bool usemask)
{

	if (usemask)
	{
		// Conversion en HSV
		Mat hsv_roi;
		cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
		// Création du masque
		Mat maskroi;
		inRange(hsv_roi, Scalar(140., 0., 0.), Scalar(255., 255., 150.), maskroi); // Créer un masque contenant 255 lorsque la valeur est dans l'interval

		// Calcul de l'histogramme
		calcHist(&roi, 1, channels, maskroi, m_roi_hist, 2, histSize, ranges, true, false);
	}
	// Sans masque (fonctionne très mal)
	else
		calcHist(&roi, 1, channels, noArray(), m_roi_hist, 2, histSize, ranges, true, false);

	// Normalisation de l'histogramme
	normalize(m_roi_hist, m_roi_hist, 0, 255, NORM_MINMAX);
}

// Sauvegarde d'une image Mat
void Tracking::savePic(Mat * im, int indice_nom) {
	std::ostringstream oss;
	oss << "./save/" << indice_nom << ".jpg";
	imwrite(oss.str(), *im);
}

/**
*  Méthode permettant de d'initialiser le filtre de Kalman
**/
void Tracking::initKalman()
{
	const int stateNum = 4;
	const int measureNum = 2;

	Mat statePost = (Mat_<float>(stateNum, 1) << m_track_window.x + m_track_window.width / 2.0,
		m_track_window.y + m_track_window.height / 2.0,
		0, 0);
	Mat transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1);

	m_KF.init(stateNum, measureNum);

	m_KF.transitionMatrix = transitionMatrix;
	m_KF.statePost = statePost;
	setIdentity(m_KF.measurementMatrix);
	setIdentity(m_KF.processNoiseCov, Scalar::all(1e-1));
	setIdentity(m_KF.measurementNoiseCov, Scalar::all(1e-3));
	setIdentity(m_KF.errorCovPost, Scalar::all(0.1));

	m_measurement = Mat::zeros(measureNum, 1, CV_32F);
}

/**
*  Méthode permettant d'obtenir l'état du filtre de Kalman
**/
Point Tracking::getCurrentKalmanState() const
{
	Mat statePost = m_KF.statePost;
	return Point(statePost.at<float>(0), statePost.at<float>(1));
}

/**
*  Méthode permettant de définir la nouvelle fenetre en fonction du point prédit par le filtre de Kalman
**/
void Tracking::setCurrentKalmanTrackWindow()
{
	int cols = m_current_frame.cols;
	int rows = m_current_frame.rows;

	m_track_window.x = m_KFCorrectCenter.x - m_track_window.width / 2;
	m_track_window.y = m_KFCorrectCenter.y - m_track_window.height / 2;

	m_track_window &= Rect(0, 0, cols, rows);

	if (m_track_window.width <= 0 || m_track_window.height <= 0) {
		int width = MIN(m_KFCorrectCenter.x, cols - m_KFCorrectCenter.x) * 2;
		int height = MIN(m_KFCorrectCenter.y, rows - m_KFCorrectCenter.y) * 2;

		m_track_window = Rect(m_KFCorrectCenter.x - width / 2, m_KFCorrectCenter.y - height / 2, width, height);
	}
}

/**
*  Méthode permettant d'éffectuer une itération d'un CamShift sur l'image suivante de la vidéo
*  channels, histSize, range et ranges : parmètres de création de l'histogramme
*  return le centre de la fenetre de tracking actuel
**/
Point Tracking::camShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat hsv, dst;

	// Récupération de l'image suivante
	m_current_frame = Mat();
	m_cap >> m_current_frame;
	// Ajout de l'image dans la classe de recherche d'oiseau
	m_geo.setPictures(m_current_frame);

	if (!m_current_frame.empty())
	{
		// Conversion de l'image en HSV 
		cvtColor(m_current_frame, hsv, COLOR_BGR2HSV);
		// Appel de la fonction de création d'une image de "BackProjection" qui renvoie une image (dst) représentant pour chaque pixel une probabilité
		// d'être sur la cible en fonction de l'histogramme créée précédement
		calcBackProject(&m_current_frame, 1, channels, m_roi_hist, dst, ranges);

		// Appel de l'algorithme de CamShift
		RotatedRect rot = CamShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));
		// Récupération du centre de la fenetre de tracking
		m_center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

		// On fait une prédiction avec le filtre de Kalman
		m_KF.predict();
		// et on la récupère
		m_KFPredictCenter = getCurrentKalmanState();

		// Récupération de la mesure du centre
		m_measurement.at<float>(0) = m_center_of_rect.x;
		m_measurement.at<float>(1) = m_center_of_rect.y;
		
		// Puis on corrige notre prédiction en fonction de la mesure
		m_KF.correct(m_measurement);
		m_KFCorrectCenter = getCurrentKalmanState();

		// Mouvement de la fenetre selon le filtre de Kalman
		setCurrentKalmanTrackWindow();

		// Si nous avons perdu la cible alors nous s'affichons pas le rectangle et le point montrant celle ci
		if (!m_isLost)
		{
			rectangle(m_current_frame, m_track_window, Scalar(255, 128, 128), 2);
			circle(m_current_frame, m_center_of_rect, 1, Scalar(0, 0, 255), 2);
		}

		// Appel de la fonction d'affichage du tracé de la trajectoire de l'oiseau en fonction de sa vitesse avec une condition permettant de savoir si nous une cible ou non
		traceRoute(m_current_frame, rot.size.width > 0 && rot.size.height > 0);

		// Si aucune cible
		if (rot.size.width == 0 && rot.size.height == 0)
		{
			m_isLost = true;
			cpt_reinit_histo++;
			// On incrémente le compteur général jusqu'à 20 si pas de cible trouvé, si c'est le cas alors on réinitialise la recherche de cible ainsi que son histogramme
			if (cpt_reinit_histo >= 20)
			{
				initializeHistogram(40, 40, channels, histSize, range, ranges);
				cpt_reinit_histo = 0;
			}
		}
		else {
			cpt_reinit_histo = 0;
			m_isLost = false;
		}

		imshow("image", m_current_frame);
	}

	return m_center_of_rect;
}

/**
*  Méthode permettant d'éffectuer une itération d'un MeanShift sur l'image suivante de la vidéo
*  channels, histSize, range et ranges : parmètres de création de l'histogramme
*  return le centre de la fenetre de tracking actuel
**/
Point Tracking::meanShiftTracking(const int channels[], const int histSize[], float range[], const float *ranges[])
{
	Mat hsv, dst;

	// Récupération de l'image suivante
	m_current_frame = Mat();
	m_cap >> m_current_frame;
	// Ajout de l'image dans la classe de recherche d'oiseau
	m_geo.setPictures(m_current_frame);

	if (!m_current_frame.empty())
	{
		// Conversion de l'image en HSV 
		cvtColor(m_current_frame, hsv, COLOR_BGR2HSV);
		// Appel de la fonction de création d'une image de "BackProjection" qui renvoie une image (dst) représentant pour chaque pixel une probabilité
		// d'être sur la cible en fonction de l'histogramme créée précédement
		calcBackProject(&m_current_frame, 1, channels, m_roi_hist, dst, ranges);

		// Appel de l'algorithme de CamShift
		int val = meanShift(dst, m_track_window, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 100, 0.01));
		// Récupération du centre de la fenetre de tracking
		m_center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

		// On fait une prédiction avec le filtre de Kalman
		m_KF.predict();
		// et on la récupère
		m_KFPredictCenter = getCurrentKalmanState();

		// Récupération de la mesure du centre
		m_measurement.at<float>(0) = m_center_of_rect.x;
		m_measurement.at<float>(1) = m_center_of_rect.y;

		// Puis on corrige notre prédiction en fonction de la mesure
		m_KF.correct(m_measurement);
		m_KFCorrectCenter = getCurrentKalmanState();

		// Mouvement de la fenetre selon le filtre de Kalman
		setCurrentKalmanTrackWindow();

		// Si nous avons perdu la cible alors nous s'affichons pas le rectangle et le point montrant celle ci
		if (!m_isLost)
		{
			rectangle(m_current_frame, m_track_window, Scalar(255, 128, 128), 2);
			circle(m_current_frame, m_center_of_rect, 1, Scalar(0, 0, 255), 2);
		}

		// Appel de la fonction d'affichage du tracé de la trajectoire de l'oiseau en fonction de sa vitesse avec une condition permettant de savoir si nous une cible ou non
		traceRoute(m_current_frame, val != 0);

		// Si aucune cible
		if (val == 0)
		{
			m_isLost = true;
			cpt_reinit_histo++;
			// On incrémente le compteur général jusqu'à 20 si pas de cible trouvé, si c'est le cas alors on réinitialise la recherche de cible ainsi que son histogramme
			if (cpt_reinit_histo >= 20)
			{
				initializeHistogram(40, 40, channels, histSize, range, ranges);
				cpt_reinit_histo = 0;
			}
		}
		else {
			cpt_reinit_histo = 0;
			m_isLost = false;
		}

		imshow("image", m_current_frame);

	}
	return m_center_of_rect;
}

/**
*  Méthode permettant l'affichage du chemin de l'oiseau
*  frame : image
*  foundTarget : booléen true si cible trouvée
**/
void Tracking::traceRoute(Mat& frame, bool foundTarget)
{
	Point center_of_rect;

	center_of_rect = Point(m_track_window.x + m_track_window.width / 2, m_track_window.y + m_track_window.height / 2);

	// Si une cible est tracké
	if (foundTarget)
	{
		// Ajout à la liste des centres de fenetres de tracking le centre actuel
		m_li_center.push_back(center_of_rect);
		// On pose une limite de 15 centres dans la liste afin de faire disparaitre la trainée de parcours derrière l'oiseau
		if (m_li_center.size() == 16) m_li_center.pop_front();
		// On retire de la liste de centre avant perte de cible le dernier centre trouvé
		if (m_li_center_afterlost.size() != 0) m_li_center_afterlost.pop_front();
	}
	else {
		// Si nous perdons la cible et que la liste de centre n'est pas vide ainsi que celle des centres après perte
		// nous supprimons les centres de la liste après perte avant de récupérer celle de la liste de centre afin d'éviter des liaisons entre les points
		// avant la première perte et la seconde
		if (m_li_center.size() != 0 && m_li_center_afterlost.size() != 0) m_li_center_afterlost.clear();
		m_li_center_afterlost.insert(m_li_center_afterlost.begin(), m_li_center.begin(), m_li_center.end());
		m_li_center.clear();
	}

	double dist = 0;
	int valcolor = 0;

	// Parcours des points avant perte s'il y en a
	if (m_li_center_afterlost.size() != 0)
	{
		Point prec_lost = m_li_center_afterlost.front();

		// Affichage sous forme de lignes avec le point précedent de couleur variant du bleu au rouge en fonction de la distance parcourue
		for (auto it = m_li_center_afterlost.begin(), end = m_li_center_afterlost.end(); it != end; it++)
		{
			if (prec_lost != *it) {
				dist = sqrt(pow((*it - prec_lost).x, 2) + pow((*it - prec_lost).y, 2));
				valcolor = dist / 30 * 255;
				if (valcolor > 255) valcolor = 255;
			}
			if (prec_lost != *it) line(frame, prec_lost, *it, Scalar(255 - valcolor, 0, valcolor), 3);
			prec_lost = *it;
		}
	}

	// Parours des centres obtenues
	if (m_li_center.size() != 0)
	{
		Point prec = m_li_center.front();

		// Affichage sous forme de lignes avec le point précedent de couleur variant du bleu au rouge en fonction de la distance parcourue
		for (auto it = m_li_center.begin(), end = m_li_center.end(); it != end; it++)
		{
			if (prec != *it) {
				dist = sqrt(pow((*it - prec).x, 2) + pow((*it - prec).y, 2));
				valcolor = dist / 30 * 255;
				if (valcolor > 255) valcolor = 255;
			}
			if (prec != *it) line(frame, prec, *it, Scalar(255 - valcolor, 0, valcolor), 3);
			prec = *it;
		}
	}
}
