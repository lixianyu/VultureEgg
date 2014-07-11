
## Communication Protocols of Vulture Egg Project

List of protocols

* E2B protocol Egg to Bridge communication protocol
* B2C protocol Bridge to Cloud communication protocol

## define

* `EGG`    - The device inside the egg.
* `BRIDGE` - The device to get info from egg and transmit to cloud.
* `CLOUD`  - The server who receive data from bridge, store data into database and show it.

## E2B protocol

### link establish

* 1. `BRIDGE` call `EGG` to connect
* 2. `EGG` accept and response
* 3. connect established.

### frame format

```text
    <data frame> ::= <package><crlf>
          <crlf> ::= <CR><LF>
            <CR> ::= 0x0D
            <LF> ::= 0x0A
       <package> ::= <package start><axis data><field end><tempture data><field end><humidity data><field end><package end>
 <package start> ::= 0x55,0xaa,0xcc
   <package end> ::= 0xaa,0x55
     <field end> ::= 0x00
          <axis> ::= <x-axis><y-axis><z-axis>
        <x-axis> ::= <axis-data>
        <y-axis> ::= <axis-data>
        <z-axis> ::= <axis-data>
     <axis-data> ::= <16bit binary, higher bit first>
 <tempture data> ::= <temp(1)><field end>...,<temp(13)><field end>
          <temp> ::= <16bit binary, higher bit first>
      <humidity> ::= <16bit binary, higher bit first>
```

### `EGG` to `BRIDGE`


### payload format


## B2C protocol


### link establish

### frame format

### payload format
