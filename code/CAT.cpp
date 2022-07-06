#include <iostream>
#include <list>
#include <set>
#include <vector>
#include <gl/glut.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int N = 5000;  // 采样点的个数 500

typedef struct Vertex
{
    double x, y;  // 点的二维坐标

    Vertex()
    {
        x = 0;
        y = 0;
    }
    Vertex(double vx, double vy) :x(vx), y(vy) {}  // 用于构造点

    bool operator==(const Vertex& v) // 重载运算符==
    {
        return (x == v.x && y == v.y);
    }

    double distance(const Vertex& v)
    {
        return sqrt(pow(v.x - x, 2) + pow(v.y - y, 2));
    }

}Vertex;

typedef struct Edge  // 用于保存边
{
    Vertex v1, v2;  // 边的两个顶点
    Edge()
    {
        v1 = Vertex(0, 0);
        v2 = Vertex(0, 0);
    }
    Edge(Vertex a,Vertex b):v1(a),v2(b){}
};

typedef struct Triangle
{
    int a, b, c;  // 三角形的三个顶点索引
    Triangle()
    {
        a = 0;
        b = 0;
        c = 0;
    }
    Triangle(int v1,int v2, int v3): a(v1),b(v2),c(v3){}

    bool operator== (const Triangle& t)  // set中插入结构体，需要重载运算符
    {
        return (a == t.a) && (b == t.b) && (c == t.c);
    }

    bool operator<(const Triangle& t)
    {
        if (a < t.a) return true;
        else if (a == t.b)
        {
            if (b < t.b) return true;
            else if (b == t.b)
            {
                if (c <= t.c) return true;
                else return false;
            }
            else return false;
        }
        else return false;
    }
}Triangle;


/*
* 初始化，获取Bounding Box包裹所有的点 
*/
void initDT(Vertex* points, int num, list<Triangle>* triangles, int height, int width)
{
    double minX, minY, maxX, maxY;
    /*minX = points[0].x;
    minY = points[0].y;
    maxX = points[0].x;
    maxY = points[0].y;
    for (int i = 1; i < num; i++)
    {
        minX = min(points[i].x, minX);
        minY = min(points[i].y, minY);
        maxX = max(points[i].x, maxX);
        maxY = max(points[i].y, maxY);
    }
    int padding = 1;
    minX = minX - padding;
    minY = minY - padding;
    maxX = maxX + padding;
    maxY = maxY + padding;*/
    minX = 0;
    minY = 0;
    maxX = width;
    maxY = height;
    points[num] = { minX,minY };
    points[num + 1] = { minX,maxY };
    points[num + 2] = { maxX,minY };
    points[num + 3] = { maxX,maxY };
    triangles->push_back(Triangle(num, num + 1, num + 2)); // 从list的末端添入两个三角形
    triangles->push_back(Triangle(num + 1, num + 2, num + 3));
}

/*
*  获取由a,b,c三点组成的三角形的外接圆的圆心
*/
Vertex circleCenter(Vertex a, Vertex b, Vertex c)
{
    Eigen::Matrix3d triangle;
    triangle << a.x, a.y, 1,
        b.x, b.y, 1,
        c.x, c.y, 1;
    Eigen::Matrix3d triangle_x, triangle_y;
    double sa = (pow(a.x, 2) + pow(a.y, 2)) / 2;
    double sb = (pow(b.x, 2) + pow(b.y, 2)) / 2;
    double sc = (pow(c.x, 2) + pow(c.y, 2)) / 2;
    triangle_x << sa, a.y, 1,
        sb, b.y, 1,
        sc, c.y, 1;
    triangle_y << a.x, sa, 1,
        b.x, sb, 1,
        c.x, sc, 1;
    double x = triangle_x.determinant() / triangle.determinant();
    double y = triangle_y.determinant() / triangle.determinant();
    Vertex center(x, y);
    return center;
}

/*
*  初始化Edge
*/
void initEdges(int edges[][N], int length)
{
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length; j++)
        {
            edges[i][j] = 0;
        }
    }
}

/*
*  获取所有外接圆包含点v的三角形
*/
list<Triangle>* getTriangleQueue(list<Triangle> *triangles, Vertex* points, int edges[][N], Vertex v)
{
    list<Triangle>::iterator iter;
    list<Triangle>* triangleQueue = new list<Triangle>();
    for (iter = triangles->begin(); iter != triangles->end();)// 找出所有外接圆包含点v的三角形
    {
        Vertex a = points[iter->a];
        Vertex b = points[iter->b];
        Vertex c = points[iter->c];
        if (v == a || v == b || v == c) break;  // 点v已被添加到DT中
        // 获取外接圆圆心
        Vertex center = circleCenter(a, b, c);
        // 获取外接圆半径
        double r = a.distance(center);
        // 判断点v在圆内(d<=r)还是圆外
        double d = v.distance(center);
        if (d <= r) // 点在圆内，删除三角形a,b,c, 连接v与a,b,c
        {
            triangleQueue->push_back(Triangle(iter->a, iter->b, iter->c));
            edges[iter->a][iter->b] += 1;  // 无向边
            edges[iter->b][iter->a] += 1;
            edges[iter->a][iter->c] += 1;
            edges[iter->c][iter->a] += 1;
            edges[iter->b][iter->c] += 1;
            edges[iter->c][iter->b] += 1;
            iter = triangles->erase(iter);
        }
        else
        {
            iter++;
        }
    }
    return triangleQueue;
}

void addVertex(list<Triangle>* triangles, list<Triangle>* triangleQueue, int edges[][N], int v_index)
{
    list<Triangle>::iterator iter;
    for (iter = triangleQueue->begin(); iter != triangleQueue->end(); iter++)
    {
        int a = iter->a; // 可能的三角形有i-a-b, i-a-c, i-b-c, 如果三角形边在edges中重复出现，不添加该三角形
        int b = iter->b;
        int c = iter->c;
        if (edges[a][b] == 1)  // edges[a][b] == edges[b][a]  如果不重复，就添加三角形
        {
            triangles->push_back(Triangle(v_index, a, b));
        }
        if (edges[b][c] == 1)
        {
            triangles->push_back(Triangle(v_index, b, c));
        }
        if (edges[a][c] == 1)
        {
            triangles->push_back(Triangle(v_index, a, c));
        }
        
    }
}

void deleteInitRect(list<Triangle>* triangles, int num)
{
    set<int> rectVertex;
    rectVertex.insert(num);
    rectVertex.insert(num + 1);
    rectVertex.insert(num + 2);
    rectVertex.insert(num + 3);
    list<Triangle>::iterator iter;
    for (iter = triangles->begin(); iter != triangles->end();)
    {
        set<int> triangle;
        triangle.insert(iter->a);
        triangle.insert(iter->b);
        triangle.insert(iter->c);
        set<int> result;
        // 求交集
        set_intersection(rectVertex.begin(), rectVertex.end(), triangle.begin(), triangle.end(), inserter(result, result.begin()));
        if (result.size() > 0)  // 交集不为空， 有点是矩形顶点，删除该面
        {
            iter = triangles->erase(iter);
        }
        else {
            iter++;
        }
    }
}

/*
*  给定顶点集，获取Delaunay Triangles
*/
list<Triangle>* DT(Vertex* points, int num, int height, int width)
 {
    cout << "进入DT" << endl;
    list<Triangle> *triangles = new list<Triangle>();  // 避免stack overflow
    
    // 初始化，获取一个矩形（由两个三角形组成），把所有点集包含在其内
    initDT(points, num, triangles, height, width);
    
    list<Triangle>::iterator iter;
    //list<Triangle> *triangleQueue = nullptr; // 保存外接圆包含点v的三角形
    int edges[N][N];  // 索引：顶点索引（构成一条边）  值：边出现的次数
    // 逐点加入DT
    for (int i = 0; i < num; i++)
    {
        Vertex v = points[i];
        // 初始化edges
        initEdges(edges, num + 4);
        list<Triangle>* triangleQueue = getTriangleQueue(triangles, points, edges, v);
        // 连接i与triangleQueue中的顶点
        addVertex(triangles, triangleQueue, edges, i);
        // 打印出现有的triangle list
        cout << i << "加入后（triangle list）：" << endl;
        for (iter = triangles->begin(); iter != triangles->end(); iter++)
        {
            cout << iter->a << " " << iter->b << " " << iter->c << endl;
        }
        //delete triangleQueue;
    }
    // 删除初始矩形的四个点，及其构成的三角形
    deleteInitRect(triangles, num);
    // 打印出现有的triangle list
    cout << "打印出triangle list:" << endl;
    for (iter = triangles->begin(); iter != triangles->end(); iter++)
    {
        cout << iter->a << " " << iter->b << " " << iter->c << endl;
    }
    return triangles;
}

/*
* 显示DT
*/
void display(Vertex* points, int num, list<Triangle> triangles, int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);  // 图形显示模式
    glutInitWindowPosition(0, 0);  // 窗口位置
    glutInitWindowSize(640, 500);  // 设置窗口大小
    glutCreateWindow("DT"); // 创建窗口，命名窗口

    glClearColor(1.0, 1.0, 1.0, 0); // 设置背景颜色
    glClear(GL_COLOR_BUFFER_BIT); // 清楚颜色缓冲

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double len = 20;
    glOrtho(-len, len, -len, len, -len, len);

    // 画点
    for (int i = 0; i < num; i++)
    {
        glColor3f(0.0, 1.0, 1.0); // 蓝色
        glPointSize(4);
        glBegin(GL_POINTS);
        glVertex2d(points[i].x, points[i].y);
        glEnd();
    }
    // 画面
    list<Triangle>::iterator iter;
    for (iter = triangles.begin(); iter != triangles.end(); iter++)
    {
        glColor3f(1.0, 0.0, 0.0); // 画笔红色
        glPointSize(6);
        glBegin(GL_LINE_LOOP);
        glVertex2d(points[iter->a].x, points[iter->a].y);
        glVertex2d(points[iter->b].x, points[iter->b].y);
        glVertex2d(points[iter->c].x, points[iter->c].y);
        glEnd();
    }

    glFlush();
    glutMainLoop();
}

/*
* 在图片上显示采样点的DT
*/
void showDTImg(Mat img, Vertex* points, int num, list<Triangle>* triangles)
{
    list<Triangle>::iterator iter;
    for (iter = triangles->begin(); iter != triangles->end(); iter++)
    {
        Point pa = Point(int(points[iter->a].x), int(points[iter->a].y));
        Point pb = Point(int(points[iter->b].x), int(points[iter->b].y));
        Point pc = Point(int(points[iter->c].x), int(points[iter->c].y));
        line(img, pa, pb, Scalar(0, 0, 255), 1, 8);  // 线段颜色，宽度，类型（8，4，CV_AA）
        line(img, pb, pc, Scalar(0, 0, 255), 1, 8);  // 线段颜色，宽度，类型（8，4，CV_AA）
        line(img, pa, pc, Scalar(0, 0, 255), 1, 8);  // 线段颜色，宽度，类型（8，4，CV_AA）
    }
    imshow("DT", img);
    waitKey(0);
}

/*
*  判断点v是否在boundary上
*  为了避免误差，将点v即点v周围的四个点全判断一遍，如果有一个在，就算在boundary上
*  （主要是避免中点距离boundary很近，但刚好不在boundary上）
*/
bool isOnBoundary(Mat binaryImg, Vertex v)
{
    int d = 2;
    if (int(binaryImg.at<uchar>(int(v.y), int(v.x))) == 0 ||
        int(binaryImg.at<uchar>(int(v.y - d), int(v.x))) == 0 ||
        int(binaryImg.at<uchar>(int(v.y), int(v.x - d))) == 0 ||
        int(binaryImg.at<uchar>(int(v.y + d), int(v.x))) == 0 ||
        int(binaryImg.at<uchar>(int(v.y), int(v.x + d))) == 0)
        return true;
    return false;
}

/*
*  2D Skeleton exraction by CAT
*/
list<Edge> chordalAxisTransform(list<Triangle>* triangles, Vertex* points, int num, Mat binaryImg)
{
    int height = binaryImg.rows; // yMax
    int width = binaryImg.cols;  // xMax
    cout << "height:" << height << endl;
    cout << "width:" << width << endl;
    list<Edge> edges;
    list<Triangle>::iterator iter;
    for (iter = triangles->begin(); iter != triangles->end(); iter++)
    {
        Vertex a = points[iter->a];
        Vertex b = points[iter->b];
        Vertex c = points[iter->c];
        // 判断三角形有几条边在boundary上：先获取每条边的中点，然后获取中点的像素值，如果像素值为黑色，说明中点在boundary上，说明该条边在boundary上
        Vertex mid_ab = Vertex((a.x + b.x) / 2, (a.y + b.y) / 2);
        Vertex mid_ac = Vertex((a.x + c.x) / 2, (a.y + c.y) / 2);
        Vertex mid_bc = Vertex((b.x + c.x) / 2, (b.y + c.y) / 2);
        cout << "mid_ab: (" << mid_ab.x << "," << mid_ab.y << ")" << endl;
        cout << "mid_ac: (" << mid_ac.x << "," << mid_ac.y << ")" << endl;
        cout << "mid_bc: (" << mid_bc.x << "," << mid_bc.y << ")" << endl;
        if (isOnBoundary(binaryImg, mid_ab) && (!isOnBoundary(binaryImg, mid_bc)) && (!isOnBoundary(binaryImg, mid_ac))) // ab在boundary上
        {
            edges.push_back(Edge(mid_ac, mid_bc));
        }
        else if ((!isOnBoundary(binaryImg, mid_ab)) && (!isOnBoundary(binaryImg, mid_bc)) && isOnBoundary(binaryImg, mid_ac))  // ac在boundary上
        {
            edges.push_back(Edge(mid_ab, mid_bc));
        }
        else if ((!isOnBoundary(binaryImg, mid_ab)) && isOnBoundary(binaryImg, mid_bc) && (!isOnBoundary(binaryImg, mid_ac)))  // bc在boundary上
        {
            edges.push_back(Edge(mid_ab, mid_ac));
        }
        else if ((!isOnBoundary(binaryImg, mid_ab)) && (!isOnBoundary(binaryImg, mid_bc)) && (!isOnBoundary(binaryImg, mid_ac)))  // ab,ac,bc都不在boundary上
        {
            Vertex mid = Vertex((mid_ab.x + mid_ac.x + mid_bc.x) / 3, (mid_ab.y + mid_ac.y + mid_bc.y) / 3);
            edges.push_back(Edge(mid, mid_ab));
            edges.push_back(Edge(mid, mid_ac));
            edges.push_back(Edge(mid, mid_bc));
        }
    }
    return edges;
}

void showCATImg(Mat img, list<Edge> edges)
{
    list<Edge>::iterator iter;
    for (iter = edges.begin(); iter != edges.end(); iter++)
    {
        //cout << "a:(" << int(iter->v1.x) << "," << int(iter->v1.y) << ")" << endl;
        //cout << "b:(" << int(iter->v2.x) << "," << int(iter->v2.y) << ")" << endl;
        Point a = Point(int(iter->v1.x), int(iter->v1.y));
        Point b = Point(int(iter->v2.x), int(iter->v2.y));
        line(img, a, b, Scalar(255, 0, 0), 1, 8);  // 线段颜色，宽度，类型（8，4，CV_AA）
    }
    imshow("CAT", img);
    waitKey(0);
}

/*
*  删除DT中不合法的三角形
*/
void deleteInvalidTriangles(list<Triangle>* triangles, Vertex* points, Mat srcImg, Mat binaryImg)
{
    Mat labelImg = Mat::zeros(srcImg.size(), CV_32S);
    int numLabels = connectedComponents(binaryImg, labelImg, 8, CV_32S); // 获取连通域
    cout << "连通域个数：" << numLabels << endl;
    int mid_row = srcImg.rows / 2;
    int mid_col = srcImg.cols / 2;
    int labelGT = labelImg.at<int>(mid_row, mid_col); // 默认图像放置在图像中间，只有一个连通域， 合法的三角形所在连通域跟中点所在连通域一样
    list<Triangle>::iterator iter;
    for (iter = triangles->begin(); iter != triangles->end();)// 找出所有外接圆包含点v的三角形
    {
        Vertex a = points[iter->a];
        Vertex b = points[iter->b];
        Vertex c = points[iter->c];
        Vertex mid = Vertex((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3);
        int label = labelImg.at<int>(int(mid.y), int(mid.x));

        if (label == labelGT) // 点在连通域内，说明该三角形合法
        {
            iter++;
        }
        else
        {
            iter = triangles->erase(iter);
        }
    }
    // 显示连通图像
    Mat dstImg = Mat::zeros(srcImg.size(), srcImg.type());
    RNG rng(12345);
    vector<Vec3b> colors(numLabels);
    colors[0] = Vec3b(0, 0, 0);
    for (int i = 1; i < numLabels; i++)
    {
        colors[i] = Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    }
    int w = srcImg.cols;
    int h = srcImg.rows;
    for (int row = 0; row < h; row++)
    {
        for (int col = 0; col < w; col++)
        {
            int label = labelImg.at<int>(row, col);
            if (label == 0) continue;
            dstImg.at<Vec3b>(row, col) = colors[label];
        }
    }
    imshow("连通区域", dstImg);
    waitKey(0);
}



int main(int argc, char** argv)
{
    //// 初始化顶点数组和三角形数组
    //Vertex points[N];
    //// 输入数据点
    //int num = 8;
    //for (int i = 0; i < num; i++)
    //{
    //    cin >> points[i].x >> points[i].y;
    //}
    //// 获取Delaunay Triangulation
    //list<Triangle> triangles = DT(points, num);
    //display(points, num + 4, triangles, argc, argv);
    Mat srcImg, grayImg, binaryImg;
    srcImg = imread("E:\\课程学习\\计算机图形学\\homework\\homework2\\imgs\\human.jpg");
    cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
    threshold(grayImg, binaryImg, 145, 255, THRESH_BINARY); // 白色：255 黑色:0 （轮廓）
    // 对图像采样： 二值化后的图像轮廓由黑色像素的组成，先获取所有黑色像素点的坐标
    Vertex *points = new Vertex[N]; // （x,y） -->   (row,col)
    int height = binaryImg.rows;
    int width = binaryImg.cols;
    cout << "binaryImg图像像素值：" << endl;
    cout << "binaryImg.rows:" << binaryImg.rows << endl;
    cout << "binaryImg.cols:" << binaryImg.cols << endl;
    cout << "srcIng图像通道:" << srcImg.channels() << endl;
    cout << "grayImg图像通道：" << grayImg.channels() << endl;
    cout << "二值图像通道：" << binaryImg.channels() << endl;
    vector<vector<Point>> contours;  // 图像轮廓 每一组vector<Point> 表示一组轮廓
    findContours(binaryImg, contours, RETR_LIST, CHAIN_APPROX_NONE);
    Mat contourImg = Mat::zeros(binaryImg.size(), CV_8U);
    drawContours(contourImg, contours, -1, Scalar(255));
    int num = 0;
    int NumLimits = N - 4;
    cout << "轮廓个数：" << contours.size() << endl;
    for (int i = 0; i < contours.size()-1; i++) // 杯子： contours.size-2
    {
        vector<Point> contour = contours[i];
        for (int j = 0; j < contour.size(); j=j+1) // 30  50
        {
            if (num >= NumLimits) break;
            points[num] = Vertex(contour[j].x, contour[j].y);
            num = num + 1;
            //cout << int(binaryImg.at<uchar>(i, j)) << " ";
        }
        if (num >= NumLimits) break;
    }

    // 图像上画点
    for (int i = 0; i < N; i++)
    {
        circle(srcImg, Point(int(points[i].x),int(points[i].y)), 2, Scalar(0, 0, 255));
    }

    list<Triangle>* triangles = DT(points, num, height, width);
     // 删除不合法的dt
    deleteInvalidTriangles(triangles, points, srcImg, binaryImg);
    
    //showDTImg(srcImg, points, num, triangles);
    list<Edge> edges = chordalAxisTransform(triangles, points, num, binaryImg);
    //cout << "打印出所有的边：" << endl;

    showCATImg(srcImg, edges);

    //delete triangles;

    /*line(binaryImg, Point(1, 1), Point(250, 250), Scalar(0, 0, 255), 5, 8);*/
    //imshow("杯子", srcImg);
    //waitKey(0); 
    //std::cout << "Hello World!\n";
    return 0;
}


