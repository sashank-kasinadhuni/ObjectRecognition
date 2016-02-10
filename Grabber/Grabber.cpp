#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <iostream>

using namespace std;
using namespace pcl;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
boost::shared_ptr<visualization::CloudViewer> viewer;
Grabber* openniGrabber;
unsigned int filesSaved = 0;
bool saveCloud(false);
string filename = "";

void printUsage(const char* programName)
    {
        cout << "Usage: " << programName << " <output_filename> "
             << endl
             <<"Saves data as a set of .pcd files which start at 0"
             <<endl
             <<"Data is saved upon pressing SPACEBAR in the visualizer window"
             <<endl;
    }
 // This function is called every time the device has new data.
void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
    {
        if (! viewer->wasStopped())
            viewer->showCloud(cloud);

        if (saveCloud)
        {
            stringstream stream;
            stream << filename << filesSaved << ".pcd";
            string fname = stream.str();
            if (io::savePCDFile(fname, *cloud, true) == 0)
            {
                filesSaved++;
                cout << "Saved " << fname << "." << endl;
            }
            else PCL_ERROR("Problem saving %s.\n", fname.c_str());

            saveCloud = false;
        }
    }
// For detecting when SPACE is pressed.
    void keyboardEventOccurred(const visualization::KeyboardEvent& event,
                          void* nothing)
    {
        if (event.getKeySym() == "space" && event.keyDown())
            saveCloud = true;
    }
// Creates, initializes and returns a new viewer.
    boost::shared_ptr<visualization::CloudViewer> createViewer()
    {
        boost::shared_ptr<visualization::CloudViewer> v
        (new visualization::CloudViewer("OpenNI viewer"));
        v->registerKeyboardCallback(keyboardEventOccurred);

        return (v);
    }

int main(int argc, char** argv){
    if(console::find_argument(argc,argv,"-h") >= 0)
    {
        printUsage(argv[0]);
        return -1;
    }
    if(argc<2){
        std::cerr<<"using default filename : Test "<<std::endl;
        filename = "Test";
    }else{
        std::cerr<<"using filename : "<<argv[1]<<std::endl;
        filename= argv[1];
    }
    openniGrabber = new OpenNIGrabber();
    if (openniGrabber == 0)
        return -1;
    boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
    boost::bind(&grabberCallback, _1);
    openniGrabber->registerCallback(f);
    viewer = createViewer();
    openniGrabber->start();
    while (! viewer->wasStopped())
            boost::this_thread::sleep(boost::posix_time::seconds(1));
    openniGrabber->stop();
}
