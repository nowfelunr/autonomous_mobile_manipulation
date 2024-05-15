from flask import Flask, jsonify, render_template
import json

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    try:
        with open('status_log.json', 'r') as file:  # Adjust the path as needed
            data = json.load(file)
    except FileNotFoundError:
        data = {'error': 'File not found'}
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True)
