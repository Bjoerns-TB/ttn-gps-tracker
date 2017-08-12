[
    {
        "id": "c3e6cabe.f1abd8",
        "type": "tab",
        "label": "GPS-Tracker"
    },
    {
        "id": "dc28e196.698f6",
        "type": "ttn message",
        "z": "c3e6cabe.f1abd8",
        "name": "",
        "app": "ddb964b8.53a578",
        "dev_id": "your-dev-id",
        "field": "",
        "x": 80,
        "y": 33,
        "wires": [
            [
                "f3bfcdda.a7378"
            ]
        ]
    },
    {
        "id": "f3bfcdda.a7378",
        "type": "json",
        "z": "c3e6cabe.f1abd8",
        "name": "",
        "x": 226.5,
        "y": 33,
        "wires": [
            [
                "3769ea0b.436466"
            ]
        ]
    },
    {
        "id": "3769ea0b.436466",
        "type": "function",
        "z": "c3e6cabe.f1abd8",
        "name": "",
        "func": "var msg1 = {};\nmsg1.payload = JSON.parse(msg.payload);\nmsg1.payload = msg1.payload.location;\nreturn msg1;",
        "outputs": 1,
        "noerr": 0,
        "x": 353.5,
        "y": 33,
        "wires": [
            [
                "bda1c0f1.de599"
            ]
        ]
    },
    {
        "id": "bda1c0f1.de599",
        "type": "geohash",
        "z": "c3e6cabe.f1abd8",
        "name": "",
        "x": 487.5,
        "y": 33,
        "wires": [
            [
                "6d902f0e.c6e39"
            ]
        ]
    },
    {
        "id": "6d902f0e.c6e39",
        "type": "influxdb out",
        "z": "c3e6cabe.f1abd8",
        "influxdb": "6de8a9.738f5758",
        "name": "",
        "measurement": "gps",
        "precision": "",
        "retentionPolicy": "",
        "x": 683,
        "y": 33,
        "wires": []
    },
    {
        "id": "ddb964b8.53a578",
        "type": "ttn app",
        "z": "",
        "appId": "your-app-id",
        "region": "eu",
        "accessKey": "your-access-key"
    },
    {
        "id": "6de8a9.738f5758",
        "type": "influxdb",
        "z": "",
        "hostname": "127.0.0.1",
        "port": "8086",
        "protocol": "http",
        "database": "gps",
        "name": "",
        "usetls": false,
        "tls": ""
    }
]
