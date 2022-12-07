#include "vision.h"

Vision::Vision()
{
    centroidAtk.y = 1.3/2*100;
    centroidDef.y = 1.3/2*100;
    centroidAtk.x = 1.6*100;
    centroidDef.x = 0.1*100;

    setCentroidAtk(centroidAtk);
    setCentroidDef(centroidDef);
}

Vision::~Vision()
{

}

void Vision::run()
{
    Timer timer;
    // Creating rinobot classes
    // Creating client pointers
    VisionClient *visionClient = new VisionClient("224.0.0.1", 10002);
    RefereeClient *refereeClient = new RefereeClient("224.5.23.2", 10003);
    ReplacerClient *replacerClient = new ReplacerClient("224.5.23.2", 10004);

    // Setting our color as BLUE at left side
    VSSRef::Color ourColor = VSSRef::Color::BLUE;
    bool ourSideIsLeft = false;

    // Desired frequency (in hz)
    float freq = 60.0;

    // Setting actuator and replacer teamColor
    replacerClient->setTeamColor(ourColor);

    double width,length;
    width = 1.3/2.0;
    length = 1.7/2.0;


    Knn.load_database();

    vector<robot> rinobot; //___

    dataState bola;

    rinobot.resize(3);
    Point2f centroidDef;
    Point2f centroidAtk;

    int robots_n;
    while(1) {
        // Start timer
        timer.start();

        // Running vision and referee clients
        visionClient->run();
        refereeClient->run();

        fira_message::Frame detection = visionClient->getLastEnvironment().frame();
        fira_message::Ball ball = detection.ball();
        if(ourColor == VSSRef::Color::BLUE){
            robots_n =  detection.robots_blue_size();
        }
        if(ourColor == VSSRef::Color::YELLOW){
            robots_n =  detection.robots_yellow_size();
        }
        if(ourColor == VSSRef::Color::BLUE){
            bola.pos.x = (length+ball.x())*100;
            bola.pos.y = (width+ball.y())*100;
        }
        if(ourColor == VSSRef::Color::YELLOW){
            bola.pos.x = fabs((-length+ball.x())*100);
            bola.pos.y = fabs((-width+ball.y())*100);
        }
        if(ourColor == VSSRef::Color::BLUE){
            bola.vel = Point2d(ball.vx()*100,ball.vy()*100);
        }
        if(ourColor == VSSRef::Color::YELLOW){
            bola.vel = Point2d(ball.vx()*100*(-1),ball.vy()*100*(-1));
        }


        if(ourColor == VSSRef::Color::BLUE){
            for (int i = 0; i < robots_n; i++) {
                fira_message::Robot robot = detection.robots_blue(i);
                Point2f pos;
                Point2d vel;
                pos.x = (length+robot.x())*100;
                pos.y = (width+robot.y())*100;//convertendo para centimetros
                double ang = ajustaAngulo(robot.orientation()*180/PI);
                vel = Point2d(robot.vx()*100,robot.vy()*100);
                rinobot[i].setPosition(pos,ang, vel); //___
            }
        }

        if(ourColor == VSSRef::Color::YELLOW){
            for (int i = 0; i < robots_n; i++) {
                fira_message::Robot robot = detection.robots_yellow(i);
                Point2f pos;
                pos.x = (fabs(-length+robot.x()))*100;
                pos.y = (fabs(-width+robot.y()))*100;//convertendo para centimetros
                double ang = ajustaAngulo(robot.orientation()*180/PI+180);
                Point2d vel = Point2d(robot.vx()*100*(-1),robot.vy()*100*(-1)); //___
                rinobot[i].setPosition(pos,ang, vel); //___
            }
        }


        setBall(bola);
        setRobots(rinobot); //___
        emit emit_info(getRobots(),getEnemy(),getCentroidDef(), getCentroidAtk(), getBall(), Knn);


        // If is kickoff, send this test frame!
        if(refereeClient->getLastFoul() == VSSRef::Foul::KICKOFF) {
            replacerClient->placeRobot(0, ourSideIsLeft ? -0.2 : 0.2, 0, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.2 : 0.2, 0.2, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.2 : 0.2, -0.2, 0);
            replacerClient->sendFrame();
        }

        // Stop timer
        timer.stop();

        // Sleep for remainingTime
        long remainingTime = (1000 / freq) - timer.getMiliSeconds();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    // Closing clients
    visionClient->close();
    refereeClient->close();
    replacerClient->close();
}

void Vision::setCentroidDef(Point2d def)
{
    centroidDef = def;
}

void Vision::setCentroidAtk(Point2d atk)
{
    centroidAtk = atk;
}

void Vision::setRobots(vector<robot> robotsPoints)
{
    robots = robotsPoints;
}

void Vision::setBall(dataState ball)
{
    this->ball = ball;
}

vector<robot> Vision::getRobots()
{
    return robots;
}

vector<robot> Vision::getEnemy()
{
    return enemy;
}


Point2d Vision::getCentroidDef()
{
    return centroidDef;
}

Point2d Vision::getCentroidAtk()
{
    return centroidAtk;
}

dataState Vision::getBall()
{
    return ball;
}



