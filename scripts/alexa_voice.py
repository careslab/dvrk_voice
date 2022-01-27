import logging
from typing import Mapping
# from urllib import request
from flask import Flask, render_template, request, jsonify
from flask_ask import Ask, statement, question, session

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16

import json


app = Flask(__name__)
ask = Ask(app, "/")

logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch
def new_game():

    welcome_msg = render_template('welcome')

    return question(welcome_msg)


@ask.intent('RunAutoCameraIntent')
def autocamera_run(run):

    #Capture the json sent from Alexa servers and parse to pull out the value instead of the synonym
    data = request.get_json()
    run = str(data['request']['intent']['slots']['run']['slotValue']['resolutions']['resolutionsPerAuthority'][0]['values'][0]['value']['name']).lower()  # I'm NEVER doing this again!
    print("RunAutoCameraIntent: " + str(run))

    #publish to run topic 
    if(run == "start"):
        run_pub.publish(True)

    elif(run == "stop"):
        run_pub.publish(False)

    #return what alexa would say when done
    say = 'Done'
    return statement(say)

@ask.intent("TrackToolIntent")
def autocamera_track(tool):
    
    #Capture the json sent from Alexa servers and parse to pull out the value instead of the synonym
    data = request.get_json()
    #print(data, '\n')      #Optionally print the json
    tool = str(data['request']['intent']['slots']['tool']['resolutions']['resolutionsPerAuthority'][0]['values'][0]['value']['name']).lower()  # I'm NEVER doing this again!
    # [0]['values'][0]['value']['name']
    print("TrackToolIntent: " + str(tool))

    #publish to track topic 
    if(tool == "right"):
        track_pub.publish("Right")

    elif(tool == "left"):
        track_pub.publish("Left")

    elif(tool == "middle"):
        track_pub.publish("Middle")

    say = 'Done'
    return statement(say)

@ask.intent("KeepToolPositionIntent")
def autocamera_keep(tool):
    
    #Capture the json sent from Alexa servers and parse to pull out the value instead of the synonym
    data = request.get_json()
    #print(data, '\n')      #Optionally print the json
    tool = str(data['request']['intent']['slots']['tool']['resolutions']['resolutionsPerAuthority'][0]['values'][0]['value']['name']).lower()  # I'm NEVER doing this again!
    # [0]['values'][0]['value']['name']
    print("KeepToolPositionIntent: " + str(tool))

    #publish to track topic 
    if(tool == "right"):
        track_pub.publish("Right")

    elif(tool == "left"):
        track_pub.publish("Left")

    elif(tool == "middle"):
        track_pub.publish("Middle")

    say = 'Done'
    return statement(say)

@ask.intent("FindMyToolsIntent")
def autocamera_findtools():

    print("FindMyToolsIntent: ")

    #publish to find tools topic
    findtools_pub.publish()

    say = 'Done'
    return statement(say)

@ask.intent("InnerOuterZoomLevelIntent", convert={'value': float})
def autocamera_innerOuterZoom(zone, value):
    
    print("InnerOuterZoomLevelIntent: " + str(zone) + ":" + str(value))

    #Check for value data types and size
    if(value >= 2 or type(value) != int or type(value) != float):
        say = 'Please pick a number between zero and two'
        return statement(say)

    #publish to inner outer zoom topic 
    if(zone == "inner"):
        innerZoom_pub.publish(value)

    elif(zone == "outer"):
        outerZoom_pub.publish(value)

    else:
        say = 'Unknown zone'
        return statement(say)

    say = 'Done'
    return statement(say)

@ask.intent("SaveEcmPositionAsIntent", convert={'name': int})
def davinci_saveEcm(name):
    
    print("SaveEcmPositionAsIntent: " + str(name))

    #Check for value data types and size
    if(type(name) != int or type(name) != float):
        say = 'Please pick a number to save the endoscope position'
        return statement(say)

    ecm_saved_positions.append(name)

    #publish to save topic 
    saveEcm_pub.publish(name)

    say = 'Done'
    return statement(say)

@ask.intent("GotoEcmPositionAsIntent", convert={'name': int})
def davinci_gotoEcm(name):
    
    print("GotoEcmPositionAsIntent: " + str(name))

    #Check for value data types and size
    if(type(name) != int or type(name) != float):
        say = 'Please pick a number to move the endoscope position'
        return statement(say)

    #Check if the value has already been saved
    if(not name in ecm_saved_positions):
        say = 'not found in saved positions'
        return statement(say)

    #publish to go to topic 
    gotoEcm_pub.publish(name)

    say = 'Done'
    return statement(say)


if __name__ == '__main__':
    rospy.init_node('dvrk_voice', anonymous=True)
    run_pub = rospy.Publisher('/assistant/autocamera/run', Bool, queue_size=1, latch=True)
    track_pub = rospy.Publisher('/assistant/autocamera/track', String, queue_size=1, latch=True)
    findtools_pub = rospy.Publisher('/assistant/autocamera/find_tools', Empty, queue_size=1, latch=True)
    innerZoom_pub = rospy.Publisher('/assistant/autocamera/inner_zoom_value', Float32, queue_size=1, latch=True)
    outerZoom_pub = rospy.Publisher('/assistant/autocamera/outer_zoom_value', Float32, queue_size=1, latch=True)
    saveEcm_pub = rospy.Publisher('/assistant/save_ecm_position', Int16, queue_size=1, latch=True)
    gotoEcm_pub = rospy.Publisher('/assistant/goto_ecm_position', Int16, queue_size=1, latch=True)

    ecm_saved_positions=[]

    app.run(debug=True)