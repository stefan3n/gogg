function sendTurnLeft()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/turnLeft/" + pwm, {method: "POST"});
}

function sendTurnRight()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/turnRight/" + pwm, {method: "POST"});
}

function sendForward()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/forward/" + pwm, {method: "POST"});
}

function sendBackward()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/backward/" + pwm, {method: "POST"});
}

function sendBreak()
{
    var host = window.location.host;
    fetch("http://" + host + "/break", {method: "POST"});
}

function sendEmergency()
{
    var host = window.location.host;
    fetch("http://" + host + "/emergency", {method: "POST"});
}

function sendEndEmergency()
{
    var host = window.location.host;
    fetch("http://" + host + "/endEmergency", {method: "POST"});
}

function sendStart()
{
    var host = window.location.host;
    fetch("http://" + host + "/start", {method: "POST"});
}

function moveOne() {
    var host = window.location.host;
    var actuator = document.getElementById("actuator").value;
    var distance;

    if (actuator === "1") {
        distance = document.getElementById("distance1").value;
    } else if (actuator === "2") {
        distance = document.getElementById("distance2").value;
    }

    fetch("http://" + host + "/moveOne/" + actuator + '/' + distance, {method: "POST"});
}

function moveBoth()
{
    var host = window.location.host;
    var distance1 = document.getElementById("distance1").value;
    var distance2 = document.getElementById("distance2").value;
    fetch("http://" + host + "/moveBoth/" + distance1 + '/' + distance2, {method: "POST"});
}

function initialPos()
{
    var host = window.location.host;
    fetch("http://" + host + "/initialPos", {method: "POST"});
}

function rotateLeft()
{
    var host = window.location.host;
    var degrees = document.getElementById("degrees").value;
    fetch("http://" + host + "/rotateLeft/" + degrees, {method: "POST"});
}

function rotateRight()
{
    var host = window.location.host;
    var degrees = document.getElementById("degrees").value;
    fetch("http://" + host + "/rotateRight/" + degrees, {method: "POST"});
}

function stopRobot()
{
    var host = window.location.host;
    fetch("http://" + host + "/stopRobot", {method: "POST"});
}

function exit()
{
    var host = window.location.host;
    fetch("http://" + host + "/exit", {method: "POST"});
}

function selfDriving()
{
    var host = window.location.host;
    fetch("http://" + host + "/autonomous", {method: "POST"});
}

function manualMode()
{
    var host = window.location.host;
    fetch("http://" + host + "/manual", {method: "POST"});
}

function calLeft()
{
    var host = window.location.host;
    var steps = document.getElementById("steps").value;
    fetch("http://" + host + "/calLeft/" + steps, {method: "POST"});
}

function calRight()
{
    var host = window.location.host;
    var steps = document.getElementById("steps").value;
    fetch("http://" + host + "/calRight/" + steps, {method: "POST"});
}

function clawRotate()
{
    var host = window.location.host;
    var deg = document.getElementById("degrees").value;
    fetch("http://" + host + "/clawRotate/" + deg, {method: "POST"});
}

function clawOpen()
{
    var host = window.location.host;
    fetch("http://" + host + "/clawOpen", {method: "POST"});
}

function clawClose()
{
    var host = window.location.host;
    fetch("http://" + host + "/clawClose", {method: "POST"});
}
