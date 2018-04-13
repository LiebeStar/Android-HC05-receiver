package com.example.liebe.android_hc05;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;

import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.OutputStream;
import java.io.InputStream;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    String message, deviceName, receive;
    byte[] bytes = new byte[16];
    Boolean isDeviceFind=false, isDeviceConnected=false, isDeviceClosed=false;

    // view variables
    Button btn_Connect, btn_Send;
    EditText edt_Device, edt_Message;
    TextView txv_Receive;

    // bluetooth variables
    BluetoothAdapter mBluetoothAdapter;
    BluetoothDevice mBluetoothDevice;
    BluetoothSocket mBluetoothSocket;
    OutputStream output;
    InputStream input;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initial();
    }


    private void initial(){
        // Bind button
        btn_Connect = (Button) findViewById(R.id.btn_connect);
        btn_Send = (Button) findViewById(R.id.btn_send);

        // Bind edit text
        edt_Device = (EditText) findViewById(R.id.device);
        edt_Message = (EditText) findViewById(R.id.message);

        // Bind text view
        txv_Receive = (TextView) findViewById(R.id.textReceive);
    }


    public void connectDevice(View v)
    {
        if (btn_Connect.getText().toString().equalsIgnoreCase("CONNECT")) {
            deviceName = edt_Device.getText().toString();

            new bluetooth_stuff().execute("Find Devices");
            new bluetooth_stuff().execute("Open Device");
        } else {
            new bluetooth_stuff().execute("Close Device");
        }
    }

    public void sendMessage(View v) throws IOException {

        if (output == null) return;

        message = edt_Message.getText().toString();
        output.write(message.getBytes());

        receiveMessage();
    }

    public void receiveMessage() throws IOException {
        String tmp;
        receive = new String();

        if( input.read(bytes) != -1 ){
            tmp = new String(bytes);
            receive+=tmp;
        }

        txv_Receive.setText("Receive:" + receive);
    }

    void find_devices() {

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        if (mBluetoothAdapter == null) {
            new bluetooth_stuff().execute("Do nothing");
        }

        if (!mBluetoothAdapter.isEnabled()) {
            Intent enBT = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enBT, 0);
        }

        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();

        if (pairedDevices.size() > 0) {
            for (BluetoothDevice d : pairedDevices) {
                if (d.getName().equalsIgnoreCase(deviceName))
                {
                    mBluetoothDevice = d;
                    isDeviceFind = true;
                    break;
                }
            }
        }
    }

    void open_device() throws IOException {

        if (mBluetoothDevice == null) {
            return;
        }

        UUID uuid = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
        //mBluetoothSocket = mBluetoothDevice.createRfcomBluetoothSocketToServiceRecord(uuid);
        mBluetoothSocket = mBluetoothDevice.createRfcommSocketToServiceRecord(uuid);
        mBluetoothSocket.connect();

        input = mBluetoothSocket.getInputStream();
        output = mBluetoothSocket.getOutputStream();

        isDeviceConnected = true;
    }

    void close_device() throws IOException {

        if (output == null || input == null) {
            return;
        }

        if (mBluetoothSocket != null) {
            input.close();
            output.close();
            mBluetoothSocket.close();

            isDeviceClosed = true;
        }
    }


    private class bluetooth_stuff extends AsyncTask<String, Void, Integer> {

        @Override
        protected Integer doInBackground(String... command) {
            if (command[0].equalsIgnoreCase("Find Devices")) {
                find_devices();
                return 1;
            }
            if (command[0].equalsIgnoreCase("Open Device")){
                try {
                    open_device();
                    return 2;
                } catch (IOException e) {
                    return 3;
                }
            }
            if (command[0].equalsIgnoreCase("Close Device")) {
                try {
                    close_device();
                    return 4;
                } catch (IOException e) {
                    return 5;
                }
            }
            return null;
        }

        @Override
        protected void onPostExecute(Integer integer) {
            switch (integer) {
                case 1:
                    if (isDeviceFind) {
                        Toast.makeText(getApplicationContext(), deviceName + " found!", Toast.LENGTH_SHORT).show();
                    }
                    break;
                case 2:
                    if (isDeviceConnected) {
                        Toast.makeText(getApplicationContext(), "Connection established to " + deviceName, Toast.LENGTH_LONG).show();
                        btn_Connect.setText("Disconnect");
                    }
                    break;
                case 3:
                    Toast.makeText(getApplicationContext(), "Cannot find the device!", Toast.LENGTH_LONG).show();
                case 4:
                    btn_Connect.setText("Connect");
                    Toast.makeText(getApplicationContext(),"Device disconnected!", Toast.LENGTH_LONG).show();
                    break;
                case 5:
                    btn_Connect.setText("Disconnect");
                    Toast.makeText(getApplicationContext(),"Error closing the connection!", Toast.LENGTH_LONG).show();
                    break;
                default:
                    break;
            }
        }
    }



}
