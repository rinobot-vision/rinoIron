#include "utils.h"


vector<Point2f> leastSquares(vector<Point2f> pos)
{
    vector<Point2f> result;
    vector<double> t;
    for(int i = 0; i < (int)pos.size(); i++)
    {
        t.push_back((double)(pos.size() - i - 1)/FPS);
    }

    MatrixXd M(pos.size(),3);
    MatrixXd Vx(pos.size(),1);
    MatrixXd Vy(pos.size(),1);

    for(int i = 0; i < (int)pos.size(); i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(j == 0){
                M(i,j) = 1;
            }
            else if(j == 1){
                M(i,j) = t[i];
            }
            else{
                M(i,j) = t[i]*t[i];
            }
        }
        Vx(i,0) = pos[i].x;
        Vy(i,0) = pos[i].y;
    }

    MatrixXd Ux = (M.transpose()*M).inverse()*(M.transpose()*Vx);
    MatrixXd Uy = (M.transpose()*M).inverse()*(M.transpose()*Vy);
    MatrixXd T(3,1), T_diff(3,1);
    Point2f aux;

    T << 1, t[0], pow(t[0],2);
    T_diff << 0, 1, 2*t[0];

    aux.x = (Ux.transpose()*T)(0,0);
    aux.y = (Uy.transpose()*T)(0,0);
    result.push_back(aux);
    aux.x = (Ux.transpose()*T_diff)(0,0);
    aux.y = (Uy.transpose()*T_diff)(0,0);
    result.push_back(aux);

    return result;
}


vector<double> leastSquares(vector<double> angle)
{
    vector<double> result;
    vector<double> t;
    for(int i = 0; i < (int)angle.size(); i++)
    {
        t.push_back((double)(angle.size() - i - 1)/FPS);
    }

    MatrixXd M(angle.size(),3);
    MatrixXd V(angle.size(),1);

    for(int i = 0; i < (int)angle.size(); i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(j == 0){
                M(i,j) = 1;
            }
            else if(j == 1){
                M(i,j) = t[i];
            }
            else{
                M(i,j) = t[i]*t[i];
            }
        }
        V(i,0) = angle[i];
    }

    MatrixXd U = (M.transpose()*M).inverse()*(M.transpose()*V);
    MatrixXd T(3,1), T_diff(3,1);

    T << 1, t[0], pow(t[0],2);
    T_diff << 0, 1, 2*t[0];

    result.push_back((U.transpose()*T)(0,0));
    result.push_back((U.transpose()*T_diff)(0,0));
    return result;
}

double angleTwoPoints(Point2f p, Point2f q)
{
    Point2f vec = Point2f(q.x - p.x,q.y - p.y);
    double angle = atan2(vec.y,vec.x)*180/PI;
    return angle;
}

double euclidean_dist(Point2f p, Point2f q)
{
    return sqrt((q.x-p.x)*(q.x-p.x) + (q.y-p.y)*(q.y-p.y));
}

double ajustaAngulo(double angle)
{

    while (angle < -180)
        angle = angle + 360;
    while (angle > 180)
        angle = angle - 360;
    return angle;
}

Point2f newPoint(Point2f pos, double th, float d) {
    Point2f P;
    P.x = pos.x + d*cos(th * M_PI/180);
    P.y = pos.x + d*sin(th * M_PI/180);
    return P;
}

vector<Point2f> createPath(dataState robot, Point2f target, double th, float C[]) {
    vector<Point2f> path = {robot.pos};
    Point2f p1, p2, p3, p4;

    p1 = newPoint(target, th, -C[1]);
    if (euclidean_dist(robot.pos, p1) < C[2] && robot.pos.x < target.x) {
        p2 = newPoint(robot.pos, robot.angle, 1);
        p3 = newPoint(p1, th, -1);
    }
    else {
        p2 = newPoint(robot.pos, robot.angle, C[0]);
        p3 = newPoint(p1, th, -C[0]);
    }

    if (robot.pos.x > target.x) {
        if (robot.pos.y > target.y) {
            p4 = newPoint(target, 135, C[3]);
        }
        else {
            p4 = newPoint(target, -135, C[3]);
        }

        path.push_back(p2);
        path.push_back(p4);
        return path;
    }

    path.push_back(p2);
    path.push_back(p3);
    path.push_back(p1);
    return path;
}

vector<vector<double>> bezier(vector<Point2f> points, float precision) {
    int N = int(1/precision);
    vector<float> t{0};
    for (int n=0; n<N; n++) {
        t.push_back(t.back() + precision);
    }

    N = int(points.size());
    vector<vector<vector<double>>> R;

    for (int i=0; i<N-1; i++) {
        vector<double> x, y;
        for (int j=0; j < int(t.size()); j++) {
            x.push_back(points[i].x * (1-t[j]) + points[i+1].x * t[j]);
            y.push_back(points[i].y * (1-t[j]) + points[i+1].y * t[j]);
        }
        R.push_back({x, y});
    }
    N--;

    while (N > 1) {
        for (int i=0; i<N-1; i++) {
            vector<double> x, y;
            for (int j=0; j < int(t.size()); j++) {
                x.push_back(R[i][0][j] * (1-t[j]) + R[i+1][0][j] * t[j]);
                y.push_back(R[i][1][j] * (1-t[j]) + R[i+1][1][j] * t[j]);
            }
            R[i] = {x, y};
        }
        N--;
    }

    return R[0];
}

double angleBezier(dataState robot, Point2f target, double th) {
    float C[6];
    for (int i=0; i<6; i++) {
        C[i] = (float) CONFIG_VAR("bezierConsts")[i];
    }

    if (th == 400) {
        th = angleTwoPoints(target, {160, 65});
    }

    vector<Point2f> path = createPath(robot, target, th, C);

    if (euclidean_dist(robot.pos, path.back()) < C[4]) {
        return angleTwoPoints(robot.pos, target);
    }

    vector<vector<double>> R = bezier(path, C[5]);
    return angleTwoPoints({R[0][0], R[1][0]}, {R[0][1], R[1][1]});
}