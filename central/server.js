// based on the example on https://www.npmjs.com/package/@abandonware/noble

const noble = require('@abandonware/noble');

const uuid_service = "1101"
const uuid_value_x = "2101"
const uuid_value_y = "2102"
const uuid_value_z = "2103"

let x_val = undefined;
let y_val = undefined;
let z_val = undefined;

let samples = [];

const printIfHaveData = () => {
  if (x_val === undefined || y_val === undefined || z_val === undefined) return;

  samples.push([x_val, y_val, z_val]);
  if (samples.length > 30) {
    samples.splice(0, 1);
  }
  if (samples.length < 11) return;

  let [x, y, z] = samples[10];
  const len = Math.sqrt(x*x + y*y + z*z);
  const xy_val = Math.sqrt(x*x + y*y);
  const angle = Math.acos(xy_val / len);

  if (Math.abs(z) > 1 && angle > 1.2) {
    console.log("VERTICAL MOVE (" + len + ", " + angle + ")");
    console.log(x, y, z);
  }

  x_val = undefined;
  y_val = undefined;
  z_val = undefined;
};

noble.on('stateChange', async (state) => {
  if (state === 'poweredOn') {
    console.log("start scanning")
    await noble.startScanningAsync([uuid_service], false);
  }
});

noble.on('discover', async (peripheral) => {
  await noble.stopScanningAsync();
  await peripheral.connectAsync();
  const {characteristics} = await peripheral.discoverSomeServicesAndCharacteristicsAsync(
    [uuid_service], 
    [uuid_value_x, uuid_value_y, uuid_value_z]
  );

  x_char = characteristics.find(c => c.uuid == uuid_value_x);
  y_char = characteristics.find(c => c.uuid == uuid_value_y);
  z_char = characteristics.find(c => c.uuid == uuid_value_z);

  x_char.subscribe();
  y_char.subscribe();
  z_char.subscribe();

  console.log("Connected!");

  x_char.on('read', (data) => {
    x_val = data.readFloatLE();
    printIfHaveData();
  });
  y_char.on('read', (data) => {
    y_val = data.readFloatLE();
    printIfHaveData();
  });
  z_char.on('read', (data) => {
    z_val = data.readFloatLE();
    printIfHaveData();
  });
});

//
// read data periodically
//
let readData = async (characteristic) => {
  const value = (await characteristic.readAsync());
  console.log(value.readFloatLE(0));

  // read data again in t milliseconds
  setTimeout(() => {
    readData(characteristic)
  }, 10);
}