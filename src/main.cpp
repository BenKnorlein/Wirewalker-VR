
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


// MinVR header
#include <api/MinVR.h>
#include "main/VREventInternal.h"
#include "tinyxml2.h"
using namespace MinVR;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Just included for some simple Matrix math used below
// This is not required for use of MinVR in general
#include <math/VRMath.h>

struct pt {
	float vertex[3];
	float color[3];
	float value;
};

std::vector< std::vector <pt> >points;

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
	MyVRApp(int argc, char** argv, const std::string& configFile) : VRApp(argc, argv, configFile), isInitialised(false),movement_y(0.0), movement_x(0.0), currentList(0){
		filename = argv[2];
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

		if (startsWith(event.getName(), "Wand0_Move")){
			controllerpose = event.getDataAsFloatArray("Transform");
		}
		if(event.getName() == "Wand_Left_Btn_Down") {
			currentList++;
			if(currentList >= list.size())currentList = 0;
		}
		if(event.getName() == "Wand_Joystick_Y_Change") {
			movement_y = event.getDataAsFloat("AnalogValue");
		}
		if(event.getName() == "Wand_Joystick_X_Change") {
			movement_x = event.getDataAsFloat("AnalogValue");
		}

		if (startsWith(event.getName(), "HTC_Controller_1"))
		{
			if(event.getInternal()->getDataIndex()->exists("/HTC_Controller_1/Pose")){
				controllerpose = event.getDataAsFloatArray("Pose");
				controllerpose = controllerpose.transpose();
			}
			if (event.getInternal()->getDataIndex()->exists("/HTC_Controller_1/State/Axis0Button_Pressed")&&
				(int) event.getInternal()->getDataIndex()->getValue("/HTC_Controller_1/State/Axis0Button_Pressed")){
				double x = event.getInternal()->getDataIndex()->getValue("/HTC_Controller_1/State/Axis0/XPos");
				double y = event.getInternal()->getDataIndex()->getValue("/HTC_Controller_1/State/Axis0/YPos");
				bool rotate = false;
				if (fabs(x) > fabs(y)) rotate = true;
				
				//if (!rotate)
				{
					VRVector3 offset = 0.1 * controllerpose * VRVector3(0, 0, y);
					VRMatrix4 trans = VRMatrix4::translation(offset);
					roompose = trans * roompose;
				}
				//else
				{
					VRMatrix4 rot = VRMatrix4::rotationY(x / 10 / CV_PI);
					roompose = rot * roompose;
				}

			}
		}

		if (event.getName() == "KbdEsc_Down") {
            shutdown();
            return;
		}
        
	}

	// Callback for rendering, inherited from VRRenderHandler
	virtual void onVRRenderGraphicsContext(const VRGraphicsState& state) {
		createDisplayList();

		if(fabs(movement_x) > 0.1 || fabs(movement_y) > 0.1){
			VRVector3 offset = 0.5 * controllerpose * VRVector3(0, 0, movement_y);
			VRMatrix4 trans = VRMatrix4::translation(offset);
			roompose = trans * roompose;

			VRMatrix4 rot = VRMatrix4::rotationY(movement_x / 10 / CV_PI);
			roompose = rot * roompose;	
		}
	}

	// Callback for rendering, inherited from VRRenderHandler
    virtual void onVRRenderGraphics(const VRGraphicsState &state) {
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glClearDepth(1.0f);
		glClearColor(0.2, 0.2, 0.3, 1.f);
		glDisable(GL_LIGHTING);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(state.getProjectionMatrix());

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(state.getViewMatrix());

    		glPushMatrix();
			glMultMatrixf(roompose.getArray());
			glCallList(list[currentList]);
		glPopMatrix();

		glPushMatrix();
			glMultMatrixf(controllerpose.getArray());
			glBegin(GL_LINES);                // Begin drawing the color cube with 6 quads
			// Back face (z = -1.0f)
			glColor3f(0.5f, 0.5f, 0.0f);     // Yellow
			glVertex3f(0.0f, 0.0f, -1.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glEnd();  // End of drawing color-cube
		glPopMatrix();
      
      
	}

	void computeCenter()
	{
		double x = 0;
		double y = 0;
		double z = 0;

		for (int i = 0; i < points[0].size(); i++)
		{
			x += points[0][i].vertex[0];
			y += points[0][i].vertex[1];
			z += points[0][i].vertex[2];
		}

		x = x / points[0].size();
		y = y / points[0].size();
		z = z / points[0].size();

		std::cerr << x << " " << y <<  " " << z << std::endl;

		roompose = VRMatrix4::translation(VRVector3(-x, -y, -z));
	}

	void drawPoints(unsigned int id)
	{
		float min = 100000000;
		float max = -100000000;
		for (int i = 0; i < points[id].size(); i++)
		{
			if(min > points[id][i].value) min = points[id][i].value;			
			if(max < points[id][i].value) max = points[id][i].value;			
		}

		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < points[id].size(); i+= 2)
		{
			unsigned char r,g,b;
			colormap_jet(points[id][i].value, min, max,r,g,b);
			glColor3ub(r,g,b);		
			glVertex3f(points[id][i].vertex[0], points[id][i].vertex[1], points[id][i].vertex[2]);
			
		}
		glEnd();
	}

	void createDisplayList()
	{
		if (!isInitialised)
		{	
			for(unsigned int i = 0; i < points.size(); i++){
				glLineWidth(30.0);
				unsigned int l = glGenLists(1);
				glNewList(l, GL_COMPILE);
				drawPoints(i);
				glEndList();
				isInitialised = true;
				list.push_back(l);
			}
		}
	}

	

protected:
	
	std::string filename;
	bool isInitialised;
	std::vector <unsigned int> list;
	VRMatrix4 controllerpose;
	VRMatrix4 roompose;
	unsigned int currentList;
	float movement_y, movement_x;
};
void addPoint(int id, float x, float y, float z, float value)
{
	if(id >= points.size()){
		std::vector <pt> pts;
		points.push_back(pts);
	}

	float scale = 300;
	float y_scale = 0.1;

	pt p;
	p.vertex[0] = scale * x ;
	p.vertex[1] = y_scale * -y;
	p.vertex[2] = scale* z ;

	p.value = value;

	points[id].push_back(p);
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
void loadDataSet(std::string file)
{
	std::ifstream fin(file.c_str());
	std::istringstream in;
	std::string line;
	int count = 0;
	while (!safeGetline(fin, line).eof())
	{
		in.clear();
		in.str(line);
		std::vector<double> tmp;
		for (double value; in >> value; comma(in))
		{
			tmp.push_back(value);
		}

		if (tmp.size() >= 4)
		{
			for(int i = 3; i < tmp.size(); i++)
				addPoint(i - 3, tmp[0], tmp[1], tmp[2], tmp[i]);
		}
		line.clear();
		tmp.clear();
		count++;
	}
	fin.close();
}

int main(int argc, char **argv) {
	loadDataSet(argv[2]);
    	MyVRApp app(argc, argv, argv[1]);
  	app.run();
	exit(0);
}

