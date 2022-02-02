#!/usr/bin/env python3

from bridge import (Actuator, Replacer, Vision, Referee, 
                        NUM_BOTS, convert_angle, Entity)

from math import pi, fmod, atan2, fabs

from utils import *
from robot import *

def main_strategy(field):
    """Sets all objetives to ball coordinates."""
    ball = field["ball"]
    objectives = [Entity(index=i) for i in range(NUM_BOTS)]
    for obj in objectives:
        obj.x = ball.x
        obj.y = ball.y

    return objectives

def smallestAngleDiff(target, source):
    """Gets the smallest angle between two points in a arch"""
    a = fmod(target + 2*pi, 2*pi) - fmod(source + 2 * pi, 2 * pi)

    if (a > pi):
        a -= 2 * pi
    else:
        if (a < -pi):
            a += 2 * pi

    return a


def controller(field, objectives):
    """
        Basic PID controller that sets the speed of each motor 
        sends robot to objective coordinate
        Courtesy of RoboCin
    """

    speeds = [{"index": i} for i in range(NUM_BOTS)]
    our_bots = field["our_bots"]

    # for each bot
    for i, s in enumerate(speeds):
        Kp = 20
        Kd = 2.5

        try:
            controller.lastError
        except AttributeError:
            controller.lastError = 0

        right_motor_speed = 0
        left_motor_speed = 0

        reversed = False
        
        objective = objectives[i]
        our_bot = our_bots[i]

        angle_rob = our_bot.a

        angle_obj = atan2( objective.y - our_bot.y, 
                            objective.x - our_bot.x )

        error = smallestAngleDiff(angle_rob, angle_obj)

        if (fabs(error) > pi / 2.0 + pi / 20.0):
            reversed = True
            angle_rob = convert_angle(angle_rob + pi)
            error = smallestAngleDiff(angle_rob, angle_obj)

        # set motor speed based on error and K constants
        error_speed = (Kp * error) + (Kd * (error - controller.lastError))
        
        controller.lastError = error

        baseSpeed = 30

        # normalize
        error_speed = error_speed if error_speed < baseSpeed else baseSpeed
        error_speed = error_speed if error_speed > -baseSpeed else -baseSpeed

        if (error_speed > 0):
            left_motor_speed = baseSpeed
            right_motor_speed = baseSpeed - error_speed
        else:
            left_motor_speed = baseSpeed + error_speed
            right_motor_speed = baseSpeed

        if (reversed):
            if (error_speed > 0):
                left_motor_speed = -baseSpeed + error_speed
                right_motor_speed = -baseSpeed
            else:
                left_motor_speed = -baseSpeed
                right_motor_speed = -baseSpeed - error_speed

        s["left"] = left_motor_speed
        s["right"] = right_motor_speed
    return speeds

if __name__ == "__main__":

    # Choose team (my robots are yellow)
    #blue-left__yellow-right
    myRobotsAreYellow = False

    # Initialize all clients
    actuator = Actuator(myRobotsAreYellow, "127.0.0.1", 20011)
    replacement = Replacer(myRobotsAreYellow, "224.5.23.2", 10004)
    vision = Vision(myRobotsAreYellow, "224.0.0.1", 10002)
    referee = Referee(myRobotsAreYellow, "224.5.23.2", 10003)


    #RINO

    freq = 60.0
    width = 1.3/2.0
    length = 1.7/2.0
    rinobot = [robot(),robot(),robot()]

    # Main infinite loop
    while True:
        referee.update()
        ref_data = referee.get_data()

        vision.update()
        field = vision.get_field_data()

        if ref_data["game_on"]:

            objectives = main_strategy(field)

            speeds = controller(field, objectives)

            actuator.send_all(speeds)
            #RINO
            ball = field["ball"]
            bola = dataState()
            if myRobotsAreYellow:
                bola.pos.x = fabs((-length+ball.x())*100)
                bola.pos.y = fabs((-width+ball.y())*100)
                bola.vel = Point2f(ball.vx()*100*(-1),ball.vy()*100*(-1))
            else:
                bola.pos.x = (length+ball.x())*100
                bola.pos.y = (width+ball.y())*100
                bola.vel = Point2f(ball.vx()*100,ball.vy()*100)

            robots = field["our_bots"]
            for i in range(NUMROBOTS):
                pos = Point2f
                if myRobotsAreYellow:
                    pos.x = (fabs(-length+robots[i].x()))*100
                    pos.y = (fabs(-width+robots[i].y()))*100 #convertendo para centimetros
                    vel = Point2f(robots[i].vx()*100*(-1),robots[i].vy()*100*(-1))
                else:
                    pos.x = (length+robots[i].x())*100
                    pos.y = (width+robots[i].y())*100 #convertendo para centimetros
                    vel = Point2f(robots[i].vx()*100,robots[i].vy()*100)
                ang = ajustaAngulo(robots[i].orientation()*180/pi+180)
                rinobot[i].setPosition(pos,ang, vel)

        elif ref_data["foul"] != 7:
            # foul behaviour
            actuator.stop()

        else:
            # halt behavior
            actuator.stop()