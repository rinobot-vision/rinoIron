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


