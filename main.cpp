#include <opencv2/viz/vizcore.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
using namespace viz;

Viz3d Window("Map");

void inintBackground()
{
    Window.setBackgroundColor(Color::black());

    WLine xaxis(Point3f(0, 0, 0), Point3f(2, 0, 0), Color::red());
    xaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("xaxis", xaxis);

    WLine yaxis(Point3f(0, 0, 0), Point3f(0, 2, 0), Color::green());
    yaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("yaxis", yaxis);

    WLine zaxis(Point3f(0, 0, 0), Point3f(0, 0, 2), Color::blue());
    zaxis.setRenderingProperty(LINE_WIDTH, 5);
    Window.showWidget("zaxis", zaxis);

    //    addCamera(Window,"gcam",MakeShiftMat(-3,-3,2),MakeRotateMat(0,0,45.0/180.0*CV_PI),true);

    //    addCamera(Window,"rcam",MakeShiftMat(0,0,1),MakeRotateMat(0,0,0),false);

    //    WGrid grid(Vec2i::all(50),Vec2d::all(1),Color::white());
    //    Window.showWidget("grid",grid);
}

class Wall
{
public:
    static int num;

    Wall(double x0, double y0, double x1, double y1)
    {
        X0 = x0;
        Y0 = y0;
        X1 = x1;
        Y1 = y1;
        WLine line(Point3d(x0, y0, 0), Point3d(x1, y1, 0), Color::red());
        line.setRenderingProperty(LINE_WIDTH, 5);
        Window.showWidget("line" + to_string(num++), line);
    }

    double X0;
    double Y0;
    double X1;
    double Y1;
};

class Line
{
public:
    double a, b, c;
    Point2d p1, p2;
    static int Lnum;
    Line(Point2d P1, Point2d P2)
    {
        p1 = P1;
        p2 = P2;

        //        WLine line(Point3d(p1.x,p1.y,0),Point3d(p2.x,p2.y,0),Color::green());
        //        line.setRenderingProperty(LINE_WIDTH,5);
        //        Window.showWidget("Line"+to_string(Lnum++),line);
    }
};

void GetLinePara(Line *l)
{
    l->a = l->p1.y - l->p2.y;
    l->b = l->p2.x - l->p1.x;
    l->c = l->p1.x * l->p2.y - l->p1.y * l->p2.x;
}

struct EData
{
    double FL;
    double FR;
    double BL;
    double BR;

    double LF;
    double LB;
    double RF;
    double RB;
};

class Robot
{
public:
    static int Rnum;

    Robot()
    {
    }

    double ScanOnce(vector<Wall *> Walls, int ang, Color color)
    {
        double Len = 99999.0;
        Point3d Fpoint = position;
        for (int j = 0; j < Walls.size(); ++j)
        {
            Wall *w = Walls[j];

            Line L1 = Line(Point2d(w->X0, w->Y0), Point2d(w->X1, w->Y1));
            Line L2 = Line(Point2d(position.x, position.y), Point2d(position.x + 0.00001 * sin((-ang + angle) / 180.0 * CV_PI), position.y + 0.00001 * cos((-ang + angle) / 180.0 * CV_PI)));

            GetLinePara(&L1);
            GetLinePara(&L2);
            double D = L1.a * L2.b - L2.a * L1.b;

            if (D == 0)
            {
                double Dis = abs(L1.c - L2.c) / sqrt(pow(L1.a, 2) + pow(L1.b, 2));
                if (Dis == 0.0)
                {
                    //cout<< "Superposition"<<endl;
                }
                else
                {
                    //cout<<i<<"Parallel"<<endl;
                }
            }
            else
            {
                Point2d p;
                p.x = (L1.b * L2.c - L2.b * L1.c) / D;
                p.y = (L1.c * L2.a - L2.c * L1.a) / D;

                if ((L1.p1.x - p.x) * (p.x - L1.p2.x) >= -0.00001 && (L1.p1.y - p.y) * (p.y - L1.p2.y) >= -0.00001)
                {
                    //if((p.x - (position.x+ sin(i/180.0*CV_PI)))*((position.x + sin(i/180.0*CV_PI)) - position.x)>=-0.00001 && (p.y - (position.y + cos(i/180.0*CV_PI)))*((position.y + cos(i/180.0*CV_PI)) - position.y)>=-0.00001){
                    if ((p.x - L2.p2.x) * (L2.p2.x - L2.p1.x) >= -0.00001 && (p.y - L2.p2.y) * (L2.p2.y - L2.p1.y) >= -0.00001)
                    {
                        Point3d diff = Point3d(p.x, p.y, 0) - position;
                        double len = sqrt(diff.x * diff.x + diff.y * diff.y);

                        if (len < Len)
                        {
                            Len = len;
                            Fpoint = Point3d(p.x, p.y, 0);
                        }
                    }
                }
            }
        }
        WLine line(position, Fpoint, color);
        line.setRenderingProperty(LINE_WIDTH, 2);
        Window.showWidget("RadarLine" + to_string(ang), line);

        return Len;
    }

    EData Scan(vector<Wall *> Walls)
    {
        EData E;
        E.FL = ScanOnce(Walls, 3, Color::blue());
        E.FR = ScanOnce(Walls, 357, Color::green());

        E.BL = ScanOnce(Walls, 177, Color::red());
        E.BR = ScanOnce(Walls, 183, Color::green());

        E.LF = ScanOnce(Walls, 87, Color::red());
        E.LB = ScanOnce(Walls, 93, Color::green());

        E.RF = ScanOnce(Walls, 273, Color::red());
        E.RB = ScanOnce(Walls, 267, Color::green());

        return E;
    }

    Point3d position = Point3d(-5, 5, 0);

    double angle = 0;
};

int Wall::num = 0;
int Line::Lnum = 0;
int Robot::Rnum = 0;

Robot *robot = new Robot();
vector<Wall *> Walls;

#define FORWARD 0
#define BACKWARD 1
#define LEFTWARD 2
#define RIGHTWARD 3

#define TURNLEFT 4
#define TURNRIGHT 5

#define FRONT 6
#define BACK 7
#define LEFT 8
#define RIGHT 9

#define BACK2 10
#define LEFT2 11

vector<Point3d> path;

void MoveMission(int DirFlag, int DisFlag, int GreatLess, double ExpectDis)
{
    while (1)
    {
        switch (DirFlag)
        {
        case FORWARD:
        {
            robot->position.y += cos(robot->angle / 180.0 * CV_PI) * 0.5;
            robot->position.x += sin(robot->angle / 180.0 * CV_PI) * 0.5;
            break;
        }
        case BACKWARD:
            robot->position.y -= cos(robot->angle / 180.0 * CV_PI) * 0.5;
            robot->position.x -= sin(robot->angle / 180.0 * CV_PI) * 0.5;
            break;
        case LEFTWARD:
            robot->position.y += cos((robot->angle - 90.0) / 180.0 * CV_PI) * 0.5;
            robot->position.x += sin((robot->angle - 90.0) / 180.0 * CV_PI) * 0.5;
            break;
        case RIGHTWARD:
            robot->position.y += cos((robot->angle + 90.0) / 180.0 * CV_PI) * 0.5;
            robot->position.x += sin((robot->angle + 90.0) / 180.0 * CV_PI) * 0.5;
            break;
        case TURNLEFT:
            robot->angle -= 1.0;
            break;
        case TURNRIGHT:
            robot->angle += 1.0;
            break;
        default:
            break;
        }

        EData e = robot->Scan(Walls);

        switch (DisFlag)
        {
        case FRONT:
            if (GreatLess)
            {
                if ((e.FL + e.FR) / 2 >= ExpectDis)
                {
                    return;
                }
            }
            else
            {
                if ((e.FL + e.FR) / 2 <= ExpectDis)
                {
                    return;
                }
            }
            break;
        case BACK:
            if (GreatLess)
            {
                if ((e.BL + e.BR) / 2 >= ExpectDis)
                {
                    return;
                }
            }
            else
            {
                if ((e.BL + e.BR) / 2 <= ExpectDis)
                {
                    return;
                }
            }
            break;
        case LEFT:
            if (GreatLess)
            {
                if ((e.LF + e.LB) / 2 >= ExpectDis)
                {
                    return;
                }
            }
            else
            {
                if ((e.LF + e.LB) / 2 <= ExpectDis)
                {
                    return;
                }
            }
            break;
        case RIGHT:
            break;

        case BACK2:
            if (GreatLess)
            {
                if ((e.BL - e.BR) >= ExpectDis)
                {
                    return;
                }
            }
            else
            {
                if ((e.BL - e.BR) <= ExpectDis)
                {
                    return;
                }
            }
            break;
        case LEFT2:
            if (GreatLess)
            {
                if ((e.LF - e.LB) >= ExpectDis)
                {
                    return;
                }
            }
            else
            {
                if ((e.LF - e.LB) <= ExpectDis)
                {
                    return;
                }
            }
            break;
        default:
            break;
        }

        path.push_back(Point3d(robot->position.x, robot->position.y, robot->position.z));
        WCloud cloud(path, Color::green());
        Window.showWidget("cloud", cloud);
        Window.spinOnce(1);
    }
}

int main()
{
    inintBackground();

    Walls.push_back(new Wall(-50, 50, 0, 50));
    Walls.push_back(new Wall(-50, 0, 0, 0));
    Walls.push_back(new Wall(-50, 50, -50, 0));
    Walls.push_back(new Wall(0, 50, 0, 0));

    Walls.push_back(new Wall(-40, 23, -40, 5));
    Walls.push_back(new Wall(-40, 27, -40, 45));

    robot->Scan(Walls);

    MoveMission(FORWARD, BACK, 1, 15);

    MoveMission(TURNLEFT, BACK2, 1, 0.01);
    MoveMission(TURNLEFT, BACK2, 0, 0.01);

    MoveMission(FORWARD, FRONT, 0, 5);
    MoveMission(LEFTWARD, LEFT, 0, 7);

    MoveMission(FORWARD, FRONT, 0, 2);
    MoveMission(BACKWARD, BACK, 0, 10);

    MoveMission(TURNLEFT, BACK2, 1, 0.01);
    MoveMission(TURNLEFT, BACK2, 0, 0.01);

    MoveMission(FORWARD, FRONT, 0, 2);
    MoveMission(BACKWARD, FRONT, 1, 10);

    MoveMission(TURNRIGHT, LEFT2, 0, 0.01);
    MoveMission(TURNRIGHT, LEFT2, 1, 0.01);
    //==
    MoveMission(FORWARD, FRONT, 0, 5);
    MoveMission(LEFTWARD, LEFT, 0, 9);

    MoveMission(FORWARD, FRONT, 0, 2);
    MoveMission(BACKWARD, BACK, 0, 20);

    MoveMission(TURNLEFT, BACK2, 1, 0.01);
    MoveMission(TURNLEFT, BACK2, 0, 0.01);

    MoveMission(FORWARD, FRONT, 0, 2);

    Window.spin();
}
