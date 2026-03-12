const LQR_MSG = 0x01;
const BONING_MSG = 0x02;
const R_COEFF_MSG = 0x03;

let socket = new WebSocket("ws://192.168.4.1:80/ws");
let statusDisplay = document.getElementById("status");

let speed = document.getElementById("speed");
let dir = document.getElementById("dir");
let boningConstantInput = document.getElementById("boning-constant");
let rightCoeffInput = document.getElementById("r-coeff");

// let data = new Uint8Array(2);



function floats_to_bytes(floats){
    return new Uint8Array(new Float32Array(floats).buffer);
}

function float_msg(msg_type, floats){
    let payload = new Uint8Array(1 + coeffs.length * 4);
    payload[0] = LQR_MSG;
    payload.set(floats_to_bytes(floats), 1);
    socket.send(payload);
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
    // handleInput();
};

dir.oninput = () => {
    // handleInput();
};

// function handleInput(){
//     let s = parseFloat(speed.value);
//     let d = parseFloat(dir.value);
//
//     let L = s * Math.min(0.5, d) * 2;
//     let R = s * Math.min(0.5, 1 - d) * 2;
//
//     data[0] = Math.floor(L * 255);
//     data[1] = Math.floor(R * 255);
//
//     socket.send(data);
// }

function submit_lqr(){
    let txt = document.getElementById("lqr-coeffs").value;
    let split = txt.split(",");
    let coeffs = split.map(x => parseFloat(x));
}
