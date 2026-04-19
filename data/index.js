const UNUSED0 = 0x01;
const UNUSED1 = 0x02;
const UNUSED2 = 0x03;
const LQR_PARAMS_MSG = 0x04;
const ON_MSG = 0x05;
const OFF_MSG = 0x06;
const CONTROL_INPUT_MSG = 0x07;

let socket = new WebSocket("ws://192.168.4.1:80/ws");
socket.binaryType="arraybuffer";
let statusDisplay = document.getElementById("status");
statusDisplay.innerText = "Connecting...";

let speed = document.getElementById("speed");
let dir = document.getElementById("dir");

let param_input = document.getElementById("lqr-params");


let telemetry_channels = ["Pos", "Ang", "Vel", "AngVel", "TauL", "RauR"];
let telemetry_channel_count = telemetry_channels.length;
let ctxs = [];
let canvases = [];

const telemetrydiv = document.getElementById("telemetry");
for(let i = 0; i < telemetry_channel_count; i++){
    let canvas = document.createElement("canvas");

    telemetrydiv.appendChild(canvas);
    telemetrydiv.appendChild(document.createElement("br"));

    canvas.style.width = "100%";
    canvas.style.height = "10em";
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    let ctx = canvas.getContext("2d");

    console.log(canvas.clientHeight);
    console.log(canvas.clientWidth);

    ctxs.push(ctx);
    canvases.push(ctx);
}

let lqr_params = {
    bonler_mass: 5.4, // 0
    wheel_mass: 2.9, // 1
    body_intertia: 0.2, // 2
    wheel_inertia: 0.02, // 3
    g: 9.82, // 4
    com_offset: 0.3, // 5
    wheel_radius: 0.085, // 6
    current_gain: 1.1, // 7
    tachometer_sign: 1.0, // 8
    accelerometer_sign: 1.0, // 9
    r_coeff: 1.4, // 10
    q_x: 5, // 11
    q_theta: 1, // 12
    q_x_dot: 1, // 13
    q_theta_dot: 1, // 14
    Kp: 2.0,
    Kw: 0.8,
    Q_matrix_scalar: 1.0,
    angVelSmoothing: 0.7,
    compFilter: 0.99,
    accSmoothing: 0.7,
    useKalman: 0,
    tauFilter: 0.6
};

for(let key in lqr_params){
    let row = document.createElement("div");
    row.className = "param-row";

    let input = document.createElement("input");
    input.type = "number";
    input.value = lqr_params[key];
    input.id = key;
    input.style.width = "5em";

    let label = document.createElement("span");
    label.innerText = " " + key;

    row.appendChild(input);
    row.appendChild(label);
    param_input.appendChild(row);

}


function floats_to_bytes(floats){
    return new Uint8Array(new Float32Array(floats).buffer);
}

function float_msg(msg_type, floats){
    let payload = new Uint8Array(1 + floats.length * 4);
    payload[0] = msg_type;
    payload.set(floats_to_bytes(floats), 1);
    console.log("Sending", payload);
    socket.send(payload);
}

function redraw_telemetry(){
    let dataPointCount = ctxs[0].canvas.width;
    let start_idx = Math.max(0, telemetry[0].length - dataPointCount);
    for(let i = 0; i < telemetry_channel_count; i++){
        let ctx = ctxs[i];
        let data = telemetry[i];
        let max = Math.max(...data);
        let min = Math.min(...data);
        ctx.clearRect(0,0,ctx.canvas.width, ctx.canvas.height);
        ctx.font = "16px Monospace";
        ctx.fillText(telemetry_channels[i], 3, 15);
        ctx.beginPath();
        ctx.moveTo(0,0);
        for(let n = 0; n < dataPointCount; n++){
            ctx.lineTo(n, (data[n + start_idx] - min) / (max - min) * ctx.canvas.height);
        }
        ctx.stroke();
    }
}

function clear_telemetry(){
    for(let i = 0; i < telemetry_channel_count; i++){
        telemetry[i] = [];
    }
}

function save_telemetry(){
    let content = "";
    for(let i = 0; i < telemetry_channel_count; i++){
        content += telemetry_channels[i] + ";";
    }
    content += "\n";
    for(let i = 0; i < telemetry[0].length; i++){
        for(let n = 0; n < telemetry_channel_count; n++){
            content += telemetry[n][i] + ";";
        }
        content += "\n";
    }

    let blob = new Blob([content], {type: "text/csv"});
    let a = document.createElement("a");
    let file = URL.createObjectURL(blob);
    a.href = file;
    a.download = "telemetry.csv";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(file);
}


socket.onopen = (e) => {
    console.log(e);
    statusDisplay.innerText = "Connected";
};


let telemetry = [];
for(let i = 0; i < telemetry_channel_count; i++){
    telemetry.push([]);
}

const logBox = document.getElementById("log");
logBox.value = "";
socket.onmessage = (e) => {
    if (typeof e.data == "string"){
        console.log(e);
        logBox.value = logBox.value += e.data + "\n";
        logBox.scrollTop = logBox.scrollHeight;
    }else if(e.data instanceof ArrayBuffer){
        // Read floats until end
        let floats = new Float32Array(e.data);
        for(let i = 0; i < floats.length; i++){
            telemetry[i%telemetry_channel_count].push(floats[i]);
        }
        // for(let i = 0; i < telemetry_channel_count; i++){
        //     let to_remove = Math.max(0, telemetry[i].length - ctxs[i].canvas.width);
        //     console.log(to_remove);
        //     telemetry[i] = telemetry[i].slice(to_remove);
        // }
        redraw_telemetry();
    }
};

socket.onclose = (e) => {
    console.log(e);
    statusDisplay.innerText = "Closed";
};

socket.onerror = (e) => {
    console.log(e);
    statusDisplay.innerText = "Error";
};

speed.oninput = () => {

};

dir.oninput = () => {

};

function send_lqr_params(){
    let values = [];
    for(let key in lqr_params){
        let input = document.getElementById(key);
        values.push(parseFloat(input.value));
    }
    console.log(values);
    float_msg(LQR_PARAMS_MSG, values);
    console.log("Sent msg");
}

function set_running(state){
    if(state){
        socket.send(new Uint8Array([ON_MSG]));
    } else {
        socket.send(new Uint8Array([OFF_MSG]));
    }
}

const touchinput = document.getElementById("touchinput");
const marker = document.getElementById("marker");

let touchx = 0;
let touchy = 0;
let touched = false;


let last_pos = [0,0];
let integrated_pos = [0,0];

function getXY(e){
    let rect = e.target.getClientRects()[0];
    let x = e.x - rect.x;
    let y = e.y - rect.y;
    let xnorm = x / rect.width;
    let ynorm = 1 - y / rect.height;

    let pos = [xnorm * 2 - 1,ynorm * 2 - 1];
    integrated_pos[0] += pos[0] - last_pos[0];
    integrated_pos[1] += pos[1] - last_pos[1];
    last_pos = pos;

    integrated_pos[0] = Math.max(-1, Math.min(1, integrated_pos[0]));
    integrated_pos[1] = Math.max(-1, Math.min(1, integrated_pos[1]));

    return integrated_pos;
}

function send_input(input){
    console.log(input);
    marker.style.left = (input[0] + 1) / 2 * 100 + "%";
    marker.style.top = (1-(input[1] + 1) / 2) * 100+ "%";
    socket.send(float_msg(CONTROL_INPUT_MSG, [input[1], input[0]]));
}


touchinput.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    touched = true;
    getXY(e);
    integrated_pos=[0,0];
    // let pos = getXY(e);
    // send_input(pos);
});

touchinput.addEventListener("pointermove", (e) => {
    if(!touched) return;
    e.preventDefault();
    let pos = getXY(e);
    send_input(pos);
});

touchinput.addEventListener("pointerup", (e) => {
    e.preventDefault();
    touched = false;
    integrated_pos = [0,0];
    send_input([0,0]);
});

document.body.addEventListener("pointerup", (e) => {
    touched = false;
    integrated_pos = [0,0];
    send_input([0,0]);
});

redraw_telemetry();