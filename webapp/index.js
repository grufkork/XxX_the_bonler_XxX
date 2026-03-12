const LQR_MSG = 0x01;
const BONING_MSG = 0x02;
const R_COEFF_MSG = 0x03;
const LQR_PARAMS_MSG = 0x04;
const ON_MSG = 0x05;
const OFF_MSG = 0x06;

let socket = new WebSocket("ws://192.168.4.1:80/ws");
let statusDisplay = document.getElementById("status");
statusDisplay.innerText = "Connecting...";

let speed = document.getElementById("speed");
let dir = document.getElementById("dir");
let boningConstantInput = document.getElementById("boning-constant");
let rightCoeffInput = document.getElementById("right-coeff");

let param_input = document.getElementById("lqr-params");


let lqr_params = {
    bonler_mass: 4,
    wheel_mass: 2.9,
    body_intertia: 1e-3,
    wheel_inertia: 1e-3,
    g: 9.82,
    com_offset: 0.05,
    wheel_radius: 0.085
};

for(let key in lqr_params){
    let input = document.createElement("input");
    input.type = "number";
    input.value = lqr_params[key];
    input.id = key;
    input.style.width = "5em";
    let label = document.createElement("span");
    label.innerText = " " + key;
    param_input.appendChild(input);
    param_input.appendChild(label);
    param_input.appendChild(document.createElement("br"));
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
    console.log("Sent");
}



socket.onopen = (e) => {
    console.log(e);
    statusDisplay.innerText = "Connected";
};

socket.onmessage = (e) => {
    console.log(e);
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

function submit_lqr(){
    let txt = document.getElementById("lqr-coeffs").value;
    let split = txt.split(",");
    let coeffs = split.map(x => parseFloat(x));
}

function send_lqr_params(){
    let values = [];
    for(let key in lqr_params){
        let input = document.getElementById(key);
        values.push(parseFloat(input.value));
    }
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

boningConstantInput.onchange = () =>{
    float_msg(BONING_MSG, [parseFloat(boningConstantInput.value)]);
}
rightCoeffInput.onchange = () =>{
    float_msg(R_COEFF_MSG, [parseFloat(boningConstantInput.value)]);
}
