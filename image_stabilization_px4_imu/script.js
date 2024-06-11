// Configuration constants
const CANVAS_WIDTH = 800;
const CANVAS_HEIGHT = 600;
const SQUARE_SIZE = 50;
const WEBSOCKET_URL = 'ws://localhost:8765';

// Canvas setup
const canvas = document.createElement('canvas');
canvas.id = 'myCanvas';
canvas.width = CANVAS_WIDTH;
canvas.height = CANVAS_HEIGHT;
document.body.appendChild(canvas);

const ctx = canvas.getContext('2d');
let restX = CANVAS_WIDTH / 2;
let restY = CANVAS_HEIGHT / 2;
let posX = restX;
let posY = restY;

// Video setup
const video = document.createElement('video');
video.width = CANVAS_WIDTH;
video.height = CANVAS_HEIGHT;
video.autoplay = true;

navigator.mediaDevices.getUserMedia({ video: true })
    .then(stream => {
        video.srcObject = stream;
        video.play();
    })
    .catch(err => {
        console.error('Error accessing the camera: ' + err);
    });

video.addEventListener('play', () => {
    requestAnimationFrame(updateCanvas);
});

// WebSocket setup
const socket = new WebSocket(WEBSOCKET_URL);
socket.onmessage = event => {
    const data = JSON.parse(event.data);
    updateSquarePosition(data.dx, data.dy);
};

// Functions
function drawSquare(x, y, color) {
    ctx.fillStyle = 'rgba(0, 0, 0, 0)'; // Transparent fill
    ctx.strokeStyle = color; // Outline color
    ctx.setLineDash([5, 3]); // Dotted line pattern
    ctx.lineWidth = 2; // Width of the outline

    // Draw the square
    ctx.fillRect(x - SQUARE_SIZE / 2, y - SQUARE_SIZE / 2, SQUARE_SIZE, SQUARE_SIZE);
    ctx.strokeRect(x - SQUARE_SIZE / 2, y - SQUARE_SIZE / 2, SQUARE_SIZE, SQUARE_SIZE);
}

function updateCanvas() {
    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT); // Clear the canvas
    ctx.drawImage(video, 0, 0, CANVAS_WIDTH, CANVAS_HEIGHT); // Draw the video frame onto the canvas

    // Draw the square
    drawSquare(posX, posY, 'green');

    requestAnimationFrame(updateCanvas);
}

function updateSquarePosition(dx, dy) {
    posX = restX + dx;
    posY = restY + dy;
}
