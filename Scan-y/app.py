from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
#@app.route('/hello/<name>')
def hello_world():
    #return 'Hello world!'
    #return render_template('index.html')
    # return render_template('main.html', name=name) #За подаване на данни към template
    
if __name__ == '__main__':
    app.run()
