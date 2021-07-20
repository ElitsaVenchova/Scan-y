from flask import Flask, render_template

app = Flask(__name__)


@app.route('/')
def hello_world():
    return render_template('main.html')
    # return render_template('main.html', cities=cities) #За подаване на данни към template


if __name__ == '__main__':
    app.run()
