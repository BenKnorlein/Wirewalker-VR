
// OpenGL platform-specific headers
#if defined(WIN32)
#define NOMINMAX
#include <windows.h>
#include <GL/gl.h>
#include <gl/GLU.h>
#elif defined(__APPLE__)
#include <OpenGL/OpenGL.h>
#include <OpenGL/glu.h>
#else
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define M_PI 3.14159265358979323846

#define GLOBAL_SCALE 0.001
#define XY_SCALE GLOBAL_SCALE 
#define DEPTH_SCALE XY_SCALE
#define DEPTH_MULTIPLIER 100
#define THICKNESS XY_SCALE *30
#define MOVE_SCALE XY_SCALE  * 250
#define MARKER_HEIGHT 0.05

// MinVR header
#include <api/MinVR.h>
#include "main/VREventInternal.h"
#include "tinyxml2.h"
#include "VRFontHandler.h"
using namespace MinVR;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Just included for some simple Matrix math used below
// This is not required for use of MinVR in general
#include <math/VRMath.h>

#include <gle.h>

struct pt {
	float vertex[3];
	float color[3];
	float value;
};
struct triangle {
	int idx[3];
};
std::vector<std::string> values_legend = {
	"Temperature (degrees C)",
	"Salinity (parts per thousand)",
	"Density (kg / m ^ 3)",
	"Chl fluorescence (relative counts)",
	"Backscattering (relative counts)",
	"Beam attenuation (m^-1)",
	"Sunlight (W / m ^ 2)"
};

std::vector<int> date [5];
std::vector<string> marker;
std::vector<double> marker_idx;
std::vector<double> min_max[2];
bool min_max_set = false;
bool has_mesh = false;
std::vector< std::vector <pt> >points[2];
std::vector< std::vector <pt> >points_mesh;
std::vector< std::vector <triangle> >triangles_mesh;

double pts[300000][3];
float col[300000][3];
float col_alpha[300000][4];
int currentScale = 0;
bool measuring = false;
bool play = false;
unsigned int currentFrame;

void parse_marker()
{
	for (int i = 1; i < date[2].size() - 1; i++)
	{
		if (date[2][i-1] != date[2][i])
		{
			marker.push_back(std::to_string(date[2][i]) + ":00");
			marker_idx.push_back(i);
		}
	}
	for (int i = 1; i < marker_idx.size() - 1; i++)
	{
		if (date[1][marker_idx[i] - 1] != date[1][marker_idx[i]])
		{
			marker[i] = std::to_string(date[0][0]) + "/" + std::to_string(date[1][1]) + " - " +marker[i];
		}
	}
}

void colormap_jet(
		double value,
		double min_val,
		double max_val,
		unsigned char &r,
		unsigned char &g,
		unsigned char &b
	){

	// scale the gray value into the range [0, 8]
	const double gray = 8 *  ((value - min_val) / (max_val - min_val));
	// s is the slope of color change
	const double s = 1.0 / 2.0;

	if (gray <= 1)
	{
		r = 0;
		g = 0;
		b = static_cast<unsigned char>((gray + 1)*s * 255 + 0.5);
	}
	else if (gray <= 3)
	{
		r = 0;
		g = static_cast<unsigned char>((gray - 1)*s * 255 + 0.5);
		b = 255;
	}
	else if (gray <= 5)
	{
		r = static_cast<unsigned char>((gray - 3)*s * 255 + 0.5);
		g = 255;
		b = static_cast<unsigned char>((5 - gray)*s * 255 + 0.5);
	}
	else if (gray <= 7)
	{
		r = 255;
		g = static_cast<unsigned char>((7 - gray)*s * 255 + 0.5);
		b = 0;
	}
	else
	{
		r = static_cast<unsigned char>((9 - gray)*s * 255 + 0.5);
		g = 0;
		b = 0;
	}
}

/** MyVRApp is a subclass of VRApp and overrides two key methods: 1. onVREvent(..)
    and 2. onVRRenderGraphics(..).  This is all that is needed to create a
    simple graphics-based VR application and run it on any display configured
    for use with MinVR.
 */
class MyVRApp : public VRApp {
public:
	MyVRApp(int argc, char** argv) : VRApp(argc, argv), isInitialised(false),movement_y(0.0), movement_x(0.0), currentList(0){
		computeCenter();
    }

    virtual ~MyVRApp() {}

	bool startsWith(std::string string1, std::string string2)
	{
		if (strlen(string1.c_str()) < strlen(string2.c_str())) return false;

		return !strncmp(string1.c_str(), string2.c_str(), strlen(string2.c_str()));
	}
	
	template <typename T> int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}

	// Callback for event handling, inherited from VRApp
	virtual void onVREvent(const VREvent &event) {
		if (event.getName() == "HTC_HMD_1"){
			if (event.getInternal()->getDataIndex()->exists("/HTC_HMD_1/State/Pose")){
				headpose = event.getInternal()->getDataIndex()->getValue("/HTC_HMD_1/State/Pose");
			}
		}
		if (event.getName() == "HTC_Controller_Right_AButton_Pressed"){
			currentList++;
			if (currentList >= list[0].size())currentList = 0;
		}

		if (event.getName() == "HTC_Controller_Right_Axis1Button_Pressed"){
			measuring = true;
		}

		if (event.getName() == "HTC_Controller_Right_Axis1Button_Released"){
			measuring = false;
		}

		//if (event.getName() == "HTC_Controller_Right_GripButton_Pressed"){
		if (event.getName() == "HTC_Controller_Right_ApplicationMenuButton_Pressed"){
			currentScale++;
			if (currentScale > 1)currentScale = 0;
		}

		if (event.getName() == "HTC_Controller_Right_Axis0Button_Pressed"){
			play = !play;
		}


		if (event.getName() == "HTC_Controller_Right")
		{
			if (event.getInternal()->getDataIndex()->exists("/HTC_Controller_Right/Pose")){
				controllerpose = event.getDataAsFloatArray("Pose");
			}
			//if (event.getInternal()->getDataIndex()->exists("/HTC_Controller_1/State/Axis0Button_Pressed") &&
			//	(int)event.getInternal()->getDataIndex()->getValue("/HTC_Controller_1/State/Axis0Button_Pressed"))
			{
				movement_x = event.getInternal()->getDataIndex()->getValue("/HTC_Controller_Right/State/Axis0/XPos");
				movement_y = event.getInternal()->getDataIndex()->getValue("/HTC_Controller_Right/State/Axis0/YPos");
			}
			//else
			//{
			//	movement_y = 0;
			//	movement_x = 0;
			//}
		}

		if (event.getName() == "KbdEsc_Down") {
            shutdown();
            return;
		}
        
	}

	void computeClosestPoint()
	{
		closestPoint = -1;
		VRPoint3 pos_tmp = roompose.inverse() * controllerpose * VRPoint3(0, 0, 0);
		VRVector3 pos = VRVector3(pos_tmp.x, pos_tmp.y, pos_tmp.z);
		VRVector3 pos2 = pos + roompose.inverse() * controllerpose * VRVector3(0, 0, -5);
		double dist = 100;
		double d = (pos2 - pos).length();
		float dist2 = 30;
		for (int i = 0; i < points[currentScale][0].size(); i++)
		{
			VRVector3 pt_vector3 = VRVector3(points[currentScale][0][i].vertex);
			double dist_tmp = (pt_vector3 - pos).cross(pt_vector3 - pos2).length() / d;

			if (dist_tmp < dist)
			{
				VRPoint3 pt_point3 = VRPoint3(pt_vector3.x, pt_vector3.y, pt_vector3.z);
				double dist2_tmp = -(controllerpose.inverse() * roompose * pt_point3).z;
				if (dist2_tmp >0){
					dist = dist_tmp;
					dist2 = dist2_tmp;
					closestPoint = i;
				}
			}
		}

		//std::cerr << closestPoint << " " << dist << std::endl;
	}

	// Callback for rendering, inherited from VRRenderHandler
	virtual void onVRRenderGraphicsContext(const VRGraphicsState& state) {

		createDisplayList();

		if(fabs(movement_x) > 0.1 || fabs(movement_y) > 0.1){
			VRVector3 offset = MOVE_SCALE * controllerpose * VRVector3(0, 0, movement_y);
			VRMatrix4 trans = VRMatrix4::translation(offset);
			roompose = trans * roompose;

			VRMatrix4 rot = VRMatrix4::rotationY(movement_x / 10/ CV_PI);
			roompose = rot * roompose;	
		}

		if (measuring && !play)
			computeClosestPoint();

		if (play)
		{
			currentFrame = currentFrame + 1;
			if (currentFrame >= points[0][0].size())
				currentFrame = 0;
				closestPoint = currentFrame;
		}
	}

	// Callback for rendering, inherited from VRRenderHandler
    virtual void onVRRenderGraphics(const VRGraphicsState &state) {

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0f);
		glClearColor(0.0, 0.0, 0.0, 1.f);
		glDisable(GL_LIGHTING);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(state.getProjectionMatrix());

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(state.getViewMatrix());

    		glPushMatrix();
			glMultMatrixf(roompose.getArray());	
			if (!play){
				glDisable(GL_CULL_FACE);
				glCallList(list[currentScale][currentList]);
			}
			else
			{
				glCallList(list_thin[currentScale][currentList]);
				drawTrace();
			}
		glPopMatrix();

		glPushMatrix();
			glMultMatrixf(controllerpose.getArray());
			glBegin(GL_LINES);                // Begin drawing the color cube with 6 quads
			// Back face (z = -1.0f)
			glColor3f(0.5f, 0.5f, 0.0f);     // Yellow
			glVertex3f(0.0f, 0.0f, -30.0f);
			glVertex3f(0.0f, 0.0f, (measuring) ? -0.65f:-0.1f);
			glEnd();  // End of drawing color-cube
		glPopMatrix();

		if (measuring)
		{
			if (closestPoint >= 0 && points[currentScale][currentList][closestPoint].value != -9999999){
					glPushMatrix();
						glMultMatrixf(controllerpose.getArray());
						glColor3f(0.7f, 0.7f, 0.7f);     // Yellow

						double width = 0.025;
						double offset = 0.15;
						double pos = 0.5 * (min_max[1][currentList] - points[currentScale][currentList][closestPoint].value) / (min_max[1][currentList] - min_max[0][currentList]);

						VRFontHandler::getInstance()->renderTextBox(std::to_string(points[currentScale][currentList][closestPoint].value),
							-0.5,
							0.0,
							-(0.5 - pos + offset),
							1.0, 0.1
							);
					

						glPushMatrix();
						glRotatef(-90, 1, 0, 0);
						glColor3f(0.5f, 0.5f, 0.5f);
						char date_str[50];
						sprintf(date_str, "%d/%d - %02d:%02d:%02d", date[0][closestPoint], date[1][closestPoint], date[2][closestPoint], date[3][closestPoint], date[4][closestPoint]);
					
						VRFontHandler::getInstance()->renderTextBox(std::string(date_str),
							-0.5,
							0.1,
							0.0,
							1.0, 0.03, VRFontHandler::CENTER);

						glPopMatrix();
					glPopMatrix();
				}	
			}

		glPushMatrix();
			glMultMatrixf(roompose.getArray());
			drawMarkers();
		glPopMatrix();

		glPushMatrix();
			glMultMatrixf(controllerpose.getArray());
			drawLegend();
		glPopMatrix();

		if (has_mesh){
			glPushMatrix();
			glMultMatrixf(roompose.getArray());
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			if(currentList == 4) 
				glCallList(list_mesh[0]);
			else if (currentList == 3)
				glCallList(list_mesh[1]);
			else if (currentList == 0)
				glCallList(list_mesh[2]);
			glDisable(GL_BLEND);
			glPopMatrix();
		}
	}

	void computeCenter()
	{
		double x = 0;
		double y = 0;
		double z = 0;

		for (int i = 0; i < points[0][0].size(); i++)
		{
			x += points[0][0][i].vertex[0];
			y += points[0][0][i].vertex[1];
			z += points[0][0][i].vertex[2];
		}

		x = x / points[0][0].size();
		y = y / points[0][0].size();
		z = z / points[0][0].size();

		//std::cerr << x << " " << y <<  " " << z << std::endl;

		roompose = VRMatrix4::translation(VRVector3(-x, -y, -z));
	}

	void drawMarkers()
	{
		for (int i = 0; i < marker.size(); i++)
		{
			VRPoint3 markerpos = roompose * VRPoint3(points[currentScale][0][marker_idx[i]].vertex[0], 0, points[currentScale][0][marker_idx[i]].vertex[2]);
			VRPoint3 headpos = headpose * VRPoint3(0, 0, 0);
			VRVector3 dir = headpos - markerpos;
			dir = roompose.inverse()  * dir;
			dir.y = 0 ;
			dir = dir.normalize();
			double angle = 180.0f / M_PI * atan2(dir.x, dir.z);

			glPushMatrix();
			glTranslatef(points[currentScale][0][marker_idx[i]].vertex[0], 0, points[currentScale][0][marker_idx[i]].vertex[2]);
			glRotatef(angle, 0, 1, 0);
			glColor3f(0.5f, 0.5f, 0.5f);
			float height = 3 + (24 - date[2][marker_idx[i]]);
			VRFontHandler::getInstance()->renderTextBox(marker[i], -1000,
				MARKER_HEIGHT *  height,
				0, 2000.0, 0.1);
			
			glBegin(GL_LINES);         
			glColor3f(0.5f, 0.5f, 0.0f);     // Yellow	
			glVertex3f(0,0,0);
			glVertex3f(0,  MARKER_HEIGHT * height, 0);
			glEnd();  // End of drawing color-cube	

			glPopMatrix();
		}
	}

	void drawLegend()
	{
		int step_length = 101;;
		double width = 0.025;
		double length = 0.5 / step_length;
		double offset = 0.15;
		if (measuring){
			double step = (min_max[1][currentList] - min_max[0][currentList]) / step_length;
			glBegin(GL_QUAD_STRIP);
			unsigned char r, g, b;
			for (int i = 0; i < step_length; i++)
			{
				colormap_jet(min_max[0][currentList] + step* i , min_max[0][currentList], min_max[1][currentList], r, g, b);
				glColor3ub(r, g, b);
				glVertex3f(-width, 0, -length*i - offset);
				glVertex3f(width, 0, -length*i - offset);
			}
			glEnd();

			glPushMatrix();
			glRotatef(-90, 1, 0, 0);
			glColor3f(0.5f, 0.5f, 0.5f);
			for (int i = 0; i < step_length; i++)
			{

				if (i % 10 == 0)
				{
					glBegin(GL_LINES);
					glVertex3f(width, length*i + offset, 0);
					glVertex3f(width + 0.025, length*i + offset, 0);
					glEnd();

					char buf[20];
					sprintf(buf, "%.3lf", min_max[0][currentList] + i * step);
					std::string text = std::string(buf);
					VRFontHandler::getInstance()->renderTextBox(text,
						width + 0.025,
						length*i + offset - 0.015,
						0,
						0.5, 0.03, VRFontHandler::LEFT);
				}
			}
			glPopMatrix();
		}

		

		glPushMatrix();
		glRotatef(-90, 1, 0, 0);
		glColor3f(0.5f, 0.5f, 0.5f);
		VRFontHandler::getInstance()->renderTextBox(values_legend[currentList],
		-0.5,
		0.05,
		0.0,
		1.0, 0.03, VRFontHandler::CENTER);

		glPopMatrix();
	}

	void drawTrace()
	{
		glEnable(GL_BLEND);
		int trace_length = 1800.0 / (date[3][1] * 60 + date[4][1] - date[3][0] * 60 - date[4][0]);
		int count = 0;
		int stop = currentFrame - trace_length;
		stop = (stop < 0) ? 0 : stop;
		for (int i = currentFrame; i >= stop; i--)
		{
			unsigned char r, g, b;
			if (points[currentScale][currentList][i].value == -9999999)
			{
				col_alpha[count][0] = 0.5f; col_alpha[count][1] = 0.5f; col_alpha[count][2] = 0.5f;
			}
			else{
				colormap_jet(points[currentScale][currentList][i].value, min_max[0][currentList], min_max[1][currentList], r, g, b);
				col_alpha[count][0] = r / 255.0f; col_alpha[count][1] = g / 255.0f; col_alpha[count][2] = b / 255.0f;
			}
			col_alpha[count][3] = 1.0 - ((float) currentFrame - i) / trace_length;

			pts[count][0] = points[currentScale][currentList][i].vertex[0];
			pts[count][1] = points[currentScale][currentList][i].vertex[1];
			pts[count][2] = points[currentScale][currentList][i].vertex[2];

			count++;
		}
		gleSetJoinStyle(TUBE_JN_ROUND);
		gleSetNumSides(9);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glePolyCylinder_c4f(count, pts, col_alpha, (currentScale == 0) ? THICKNESS * 0.5 : THICKNESS);
		glDisable(GL_BLEND);
	}

	void drawTriangles(int id)
	{
		float min = 100000000;
		float max = -100000000;

		for (int i = 0; i < points_mesh[id].size(); i++)
		{
			if (points_mesh[id][i].value != -9999999 && min > points_mesh[id][i].value) min = points_mesh[id][i].value;
			if (points_mesh[id][i].value != -9999999 && max < points_mesh[id][i].value) max = points_mesh[id][i].value;
		}

		unsigned char r, g, b;
		for (int i = 0; i < points_mesh[id].size(); i++)
		{
			/*if (id == 0)
				colormap_jet(points_mesh[id][i].value, min_max[0][4], min_max[1][4], r, g, b);
			else if (id == 1)
				colormap_jet(points_mesh[id][i].value, min_max[0][3], min_max[1][3], r, g, b);
			else if (id == 2)
				colormap_jet(points_mesh[id][i].value, min_max[0][0], min_max[0][1], r, g, b);
			*/
			colormap_jet(points_mesh[id][i].value, min, max, r, g, b);
			
			points_mesh[id][i].color[0] = ((float)r) / 255.0;
			points_mesh[id][i].color[1] = ((float)g) / 255.0 ;
			points_mesh[id][i].color[2] = ((float)b) / 255.0;
		}

		glBegin(GL_TRIANGLES);
		for (int i = 0; i < triangles_mesh[id].size(); i++)
		{
			glColor4f(points_mesh[id][triangles_mesh[id][i].idx[0]].color[0], points_mesh[id][triangles_mesh[id][i].idx[0]].color[1], points_mesh[id][triangles_mesh[id][i].idx[0]].color[2],0.7);
			glVertex3d(points_mesh[id][triangles_mesh[id][i].idx[0]].vertex[0], points_mesh[id][triangles_mesh[id][i].idx[0]].vertex[1], points_mesh[id][triangles_mesh[id][i].idx[0]].vertex[2]);
			glColor4f(points_mesh[id][triangles_mesh[id][i].idx[1]].color[0], points_mesh[id][triangles_mesh[id][i].idx[1]].color[1], points_mesh[id][triangles_mesh[id][i].idx[1]].color[2], 0.7);
			glVertex3d(points_mesh[id][triangles_mesh[id][i].idx[1]].vertex[0], points_mesh[id][triangles_mesh[id][i].idx[1]].vertex[1], points_mesh[id][triangles_mesh[id][i].idx[1]].vertex[2]);
			glColor4f(points_mesh[id][triangles_mesh[id][i].idx[2]].color[0], points_mesh[id][triangles_mesh[id][i].idx[2]].color[1], points_mesh[id][triangles_mesh[id][i].idx[2]].color[2], 0.7);
			glVertex3d(points_mesh[id][triangles_mesh[id][i].idx[2]].vertex[0], points_mesh[id][triangles_mesh[id][i].idx[2]].vertex[1], points_mesh[id][triangles_mesh[id][i].idx[2]].vertex[2]);

		}
		glEnd();
	}

	void drawPoints(unsigned int id, unsigned int scale, bool thin)
	{
		std::cerr << "Draw Points " << id << " " << scale << std::endl;
		float min = 100000000;
		float max = -100000000;
		if (!min_max_set){
			for (int i = 0; i < points[scale][id].size(); i++)
			{
				if (points[scale][id][i].value != -9999999 && min > points[scale][id][i].value) min = points[scale][id][i].value;
				if (points[scale][id][i].value != -9999999 && max < points[scale][id][i].value) max = points[scale][id][i].value;
			}
			if (!thin && scale == 0){
				min_max[0].push_back(min);
				min_max[1].push_back(max);
			}
		}
		else
		{
			min = min_max[0][id];
			max = min_max[1][id];
		}

		//glBegin(GL_LINE_STRIP);
		//for (int i = 0; i < points[scale][id].size(); i+= 2)
		//{
		//	unsigned char r,g,b;
		//	colormap_jet(points[scale][id][i].value, min, max,r,g,b);
		//	glColor3ub(r,g,b);		
		//	glVertex3f(points[scale][id][i].vertex[0], points[scale][id][i].vertex[1], points[scale][id][i].vertex[2]);
		//	
		//}
		//glEnd();

		for (int i = 0; i < points[scale][id].size(); i++)
			{
				unsigned char r,g,b;
				if (points[scale][id][i].value == -9999999)
				{
					col[i][0] = 0.5f; col[i][1] = 0.5f; col[i][2] = 0.5f;
				}
				else{
					colormap_jet(points[scale][id][i].value, min, max, r, g, b);
					col[i][0] = r / 255.0f; col[i][1] = g / 255.0f; col[i][2] = b / 255.0f;
				}
				pts[i][0] = points[scale][id][i].vertex[0];
				pts[i][1] = points[scale][id][i].vertex[1];
				pts[i][2] = points[scale][id][i].vertex[2];
			}
		gleSetJoinStyle(TUBE_JN_RAW);
			//std::cerr << gleGetNumSides() << std::endl;
		gleSetNumSides(6);
		glePolyCylinder(points[scale][id].size(), pts, col, (thin) ? ((scale == 0) ? THICKNESS * 0.5 *0.025 : THICKNESS *0.025) : ((scale == 0) ? THICKNESS * 0.5 : THICKNESS));
	}

	void createDisplayList()
	{
		if (!isInitialised)
		{	
			for (unsigned int scale = 0; scale < 2; scale++){
				for (unsigned int i = 0; i < points[scale].size(); i++){
					//glLineWidth(30.0);
					unsigned int l = glGenLists(1);
					glNewList(l, GL_COMPILE);
					drawPoints(i, scale, false);
					glEndList();
					
					list[scale].push_back(l);

					l = glGenLists(1);
					glNewList(l, GL_COMPILE);
					drawPoints(i, scale, true);
					glEndList();

					list_thin[scale].push_back(l);
				}
			}
			for (unsigned int i = 0; i < points_mesh.size(); i++){
				//glLineWidth(30.0);
				unsigned int l = glGenLists(1);
				glNewList(l, GL_COMPILE);
				drawTriangles(i);
				glEndList();

				list_mesh.push_back(l);
			}
			isInitialised = true;
		}
	}

	

protected:
	
	bool isInitialised;
	std::vector <unsigned int> list[2];
	std::vector <unsigned int> list_thin[2];
	std::vector <unsigned int> list_mesh;
	VRMatrix4 controllerpose;
	VRMatrix4 roompose;
	VRMatrix4 headpose;
	unsigned int currentList;
	float movement_y, movement_x;
	int closestPoint;
};

void addMeshPoint(int id, float lat, float lon, float value)
{
	if (id >= points_mesh.size()){
		std::vector <pt> pts;
		points_mesh.push_back(pts);
	}

	float scale = XY_SCALE;
	float y_scale = DEPTH_SCALE;

	double x = lat;
	double z = lon;

	pt p;
	p.vertex[0] = scale * x;
	p.vertex[1] = 0;
	p.vertex[2] = scale* z;

	p.value = value;
	points_mesh[id].push_back(p);
}

void addPoint(int id, float lat, float y, float lon, float value)
{
	if(id >= points[0].size()){
		std::vector <pt> pts;
		points[0].push_back(pts);
	}
	if (id >= points[1].size()){
		std::vector <pt> pts;
		points[1].push_back(pts);
	}

	float scale = XY_SCALE;
	float y_scale = DEPTH_SCALE;

	double x = lat;
	double z = lon;

	pt p;
	p.vertex[0] = scale * x ;
	p.vertex[1] = y_scale * - y;
	p.vertex[2] = scale* z ;

	p.value = value;
	points[0][id].push_back(p);
	p.vertex[1] = p.vertex[1] * DEPTH_MULTIPLIER ;
	points[1][id].push_back(p);
}

std::istream& safeGetline(std::istream& is, std::string& t)
{
	t.clear();

	// The characters in the stream are read one-by-one using a std::streambuf.
	// That is faster than reading them one-by-one using the std::istream.
	// Code that uses streambuf this way must be guarded by a sentry object.
	// The sentry object performs various tasks,
	// such as thread synchronization and updating the stream state.

	std::istream::sentry se(is, true);
	std::streambuf* sb = is.rdbuf();

	for (;;)
	{
		int c = sb->sbumpc();
		switch (c)
		{
		case '\n':
			return is;
		case '\r':
			if (sb->sgetc() == '\n')
				sb->sbumpc();
			return is;
		case EOF:
			// Also handle the case when the last line has no line ending
			if (t.empty())
			{
				is.setstate(std::ios::eofbit);
			}
			return is;
		default:
			t += (char)c;
		}
	}
}

std::istream& comma(std::istream& in)
{
	if ((in >> std::ws).peek() != std::char_traits<char>::to_int_type(','))
	{
		in.setstate(std::ios_base::failbit);
	}
	return in.ignore();
}

void load_min_max(std::string file)
{
	std::ifstream fin(file.c_str());
	std::istringstream in;
	std::string line;
	bool good = true;
	while (!safeGetline(fin, line).eof() && good)
	{
		in.clear();
		in.str(line);
		std::vector<double> tmp;
		for (double value; in >> value; comma(in))
		{
			tmp.push_back(value);
		}

		for (int i = 0; i < 2; i++)
			min_max[i].push_back(tmp[i]);

		line.clear();
		tmp.clear();
	}
	fin.close();

}

void loadMeshPoints(std::string file, int id)
{
	std::ifstream fin(file.c_str());
	std::istringstream in;
	std::string line;
	int count = 0;
	bool good = true;
	while (!safeGetline(fin, line).eof() && good)
	{
		in.clear();
		in.str(line);
		std::vector<double> tmp;
		for (double value; in >> value; comma(in))
		{
			tmp.push_back(value);
		}

		
		addMeshPoint(id, tmp[0], tmp[1], tmp[2]);

		line.clear();
		tmp.clear();
		count++;		
	}
	fin.close();
}

void loadTriangles(std::string file, int id)
{
	if (id >= triangles_mesh.size()){
		std::vector <triangle> tri;
		triangles_mesh.push_back(tri);
	}

	std::ifstream fin(file.c_str());
	std::istringstream in;
	std::string line;
	int count = 0;
	bool good = true;
	while (!safeGetline(fin, line).eof() && good)
	{
		in.clear();
		in.str(line);
		std::vector<int> tmp;
		for (int value; in >> value; comma(in))
		{
			tmp.push_back(value);
		}

		
		triangle p;
		p.idx[0] = tmp[0];
		p.idx[1] = tmp[1];
		p.idx[2] = tmp[2];

		triangles_mesh[id].push_back(p);

		line.clear();
		tmp.clear();
		count++;		
	}

	fin.close();
}

void loadDataSet(std::string file, int skip = 0)
{
	std::ifstream fin(file.c_str());
	std::istringstream in;
	std::string line;
	int count = 0;
	bool good = true;
	while (!safeGetline(fin, line).eof() && good)
	{
		in.clear();
		in.str(line);
		std::vector<double> tmp;
		for (double value; in >> value; comma(in))
		{
			tmp.push_back(value);
		}

		for (int i = 0; i < 5; i++)
			date[i].push_back(tmp[i+3]);


		for (int i = 8; i < tmp.size(); i++)			
			addPoint(i - 8, tmp[0], tmp[1], tmp[2], tmp[i]);			
	
		line.clear();
		tmp.clear();
		count++;
		for (int i = 0; i < skip; i++)
		{
			if (safeGetline(fin, line).eof()){
				good = false;
				break;
			}
		}
	}
	std::cerr << count << std::endl;
	fin.close();
}

int main(int argc, char **argv) {
	int skip = 0;
	
	if (argc > 5)
	{
		loadMeshPoints(argv[4],0);
		loadTriangles(argv[5], 0);

		loadMeshPoints(argv[6],1);
		loadTriangles(argv[7], 1);

		loadMeshPoints(argv[8],2);
		loadTriangles(argv[9], 2);
		has_mesh = true;
	}

	loadDataSet(argv[3], skip);
	parse_marker();
    	MyVRApp app(argc, argv);
  	app.run();
	exit(0);
}

