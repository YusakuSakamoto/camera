#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class MyData
{
public:
    MyData() : A(0), X(0), id(){}
    explicit MyData(int) : A(97), X(CV_PI), id("mydata1234"){}
    void write(FileStorage& fs) const
    {
        fs << "{" << "A" << A << "X" << X << "id" << id << "}";
    }
    void read(const FileNode& node) 
    {
        A = (int)node["A"];
        X = (double)node["X"];
        id = (string)node["id"];
    }
public:
    int A;
    double X;
    string id;
};

static void write(FileStorage& fs, const std::string&, const MyData& x)
{
    x.write(fs);
}
static void read(const FileNode& node, MyData& x, const MyData& default_value = MyData()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

int main(int ac, char** av)
{
  string filename = "./data/file.yml";
  { //write
	Mat R = Mat_<uchar>::eye(3, 3),
	  T = Mat_<double>::zeros(3, 1);
	MyData m(1);
	FileStorage fs(filename, FileStorage::WRITE);

	fs << "R" << R;
	fs << "T" << T;
	fs.release();
	cout << "Write Done." << endl;
  }

  {
	cout << endl << "Reading: " << endl;
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	MyData m;
	Mat R, T;

	fs["R"] >> R;
	fs["T"] >> T;
	fs["MyData"] >> m;

	cout << endl
		 << "R = " << R << endl;
	cout << "T = " << T << endl << endl;

  }
  return 0;
}
