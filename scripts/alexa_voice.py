import logging

from flask import Flask, render_template

from flask_ask import Ask, statement, question, session


app = Flask(__name__)

ask = Ask(app, "/")

logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch

def new_game():

    welcome_msg = render_template('welcome')

    return question(welcome_msg)



@ask.intent("RunAutoCameraIntent")

def answer(run):
    print(run)
    print('\n')

    if run == 'start':

        msg = 'Starting auto camera'
    
    elif run == 'stop':

        msg = 'Starting auto camera'

    else:

        msg = 'Unknown variable'

    return statement(msg)


if __name__ == '__main__':

    app.run(debug=True)