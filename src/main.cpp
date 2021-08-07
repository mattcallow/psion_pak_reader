/* 
 * PSION II PAK reader
 * Will also program EPROM PAKs
 */
#include <Arduino.h>
#include <TimerOne.h>
#include <digitalWriteFast.h>
// define ZERO_COUNT if you want to include a count for 0 bits (may be useful to determine if UV erase is working)
#undef ZERO_COUNT
// define TEST_PROG if you want to include code for test programming
#undef TEST_PROG

const byte data_bus[] ={2,3,4,5,6,7,8,9};

const byte SMR = A2;
const byte SCLK = A0;
const byte SOE_B = A1;
const byte SS_B = 13;
const byte SPGM_B = 12;
const byte PWMA = 10;
const byte VPP_IN = A3;

TimerOne timer1;

uint32_t current_address = 0;
boolean in_srec = false;

#pragma pack(1)
typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint16_t frame;
} standard_header_t;

typedef struct {
  uint8_t code;
  uint8_t id;
  uint8_t version;
  uint8_t priority;
  uint16_t address;
} bootable_header_t;

typedef struct {
  uint8_t flags;
  uint8_t size;
  union {
    standard_header_t standard;
    bootable_header_t boot;
  };
  uint16_t checksum; 
} pak_header_t;
#pragma pack()

pak_header_t pak_header;
uint32_t eprom_size = (uint32_t)0;

void print_error(const char *, Stream &);

void set_read() {
  for (byte i=0;i<sizeof(data_bus);i++) {
    pinMode(data_bus[i], INPUT);
  }
}

void set_write() {
  for (byte i=0;i<sizeof(data_bus);i++) {
    pinMode(data_bus[i], OUTPUT);
  }
}

void bus_idle() {
  digitalWrite(SMR, HIGH);
  digitalWrite(SCLK, LOW);
  digitalWrite(SOE_B, HIGH);
  digitalWrite(SS_B, HIGH);
  digitalWrite(SPGM_B, HIGH);
}

void incr_addr(int32_t num) {
  while (num-->0) {
    if (digitalRead(SCLK)) {
      digitalWrite(SCLK, LOW);
    } else {
      digitalWrite(SCLK, HIGH);
    }
    current_address++;
    if ((pak_header.flags & 4) && ((current_address & 0xff) == 0)) {
      // paged ROM - toggle page address. Clock occurs on low to high transition
      // SPGM must be LOW for programming
      digitalWrite(SPGM_B, HIGH);
      digitalWrite(SPGM_B, LOW);
    }
  }
}

void inline incr_addr() { incr_addr(1); }

void set_address(uint32_t addr) {
  digitalWrite(SMR, HIGH); // assert reset
  delayMicroseconds(100);
  digitalWrite(SCLK, LOW);
  digitalWrite(SPGM_B, LOW);
  current_address = 0;
  digitalWrite(SMR, LOW); // disable reset

  if (pak_header.flags & 4) {
    // paged ROM
    uint32_t page = addr >> 8;
    while(page-->0) {
      // paged ROM - toggle page address. Clock occurs on low to high transition
      // SPGM must be LOW for programming
      digitalWrite(SPGM_B, HIGH);
      digitalWrite(SPGM_B, LOW);
    }
    current_address = addr & 0xff00;
    uint32_t num = addr & 0xff;
    while (num-->0) {
      if (digitalRead(SCLK)) {
        digitalWrite(SCLK, LOW);
      } else {
        digitalWrite(SCLK, HIGH);
      }
      current_address++;
    }
  } else {
    // linear ROM
    incr_addr(addr);
  }
}

uint8_t read_data_bus() {
  uint8_t ret = 0;
  set_read();
  for (byte i=0;i<sizeof(data_bus);i++) {
    if (digitalRead(data_bus[i])) ret |= 1<<i;
  }
  return ret;
}

void write_data_bus(uint8_t data) {
  set_write();
  for (byte i=0;i<sizeof(data_bus);i++) {
    if (data & (1<<i)) {
      digitalWrite(data_bus[i], HIGH);
    } else {
      digitalWrite(data_bus[i], LOW);
    }
  }
}

uint8_t read_next_byte() {
  incr_addr();
  set_read();
  digitalWrite(SOE_B, LOW); // output enable
  digitalWrite(SS_B, LOW); // Slot enable
  uint8_t ret = read_data_bus();
  digitalWrite(SS_B, HIGH); 
  digitalWrite(SOE_B, HIGH); 
  return ret;
}

uint8_t read_byte(uint32_t addr) {
  set_address(addr);
  set_read();
  digitalWrite(SOE_B, LOW); // output enable
  digitalWrite(SS_B, LOW); // Slot enable
  uint8_t ret = read_data_bus();
  digitalWrite(SS_B, HIGH); 
  digitalWrite(SOE_B, HIGH); 
  return ret;
}

void print_pak_header(pak_header_t *h) {
  Serial.println();
  Serial.print("Flags: 0x");Serial.println(h->flags, 16);
  Serial.print("0:");Serial.println((h->flags & 0x01)?"invalid":"valid");
  Serial.print("1:");Serial.println((h->flags & 0x02)?"datapak":"rampak");
  Serial.print("2:");Serial.println((h->flags & 0x04)?"paged":"linear");
  Serial.print("3:");Serial.println((h->flags & 0x08)?"not write protected":"write protected");
  Serial.print("4:");Serial.println((h->flags & 0x10)?"non-bootable":"bootable");
  Serial.print("5:");Serial.println((h->flags & 0x20)?"copyable":"copy protected");
  Serial.print("6:");Serial.println((h->flags & 0x40)?"standard":"flashpak or debug RAM pak");
  Serial.print("7:");Serial.println((h->flags & 0x80)?"MK1":"MK2");
  Serial.print("Size: "); Serial.println(eprom_size);
}

void read_dir() {
    Serial.println(F("TYPE         NAME     ID   D SIZE"));
    read_byte(9);
    while(current_address <eprom_size) {
      char filename[9];
      uint8_t b1 = read_next_byte();
      if (b1 == 0xff) break;
      uint8_t rec_type = read_next_byte();
      uint16_t length;
      if (rec_type == 0x80) {
        length = 256*read_next_byte() + read_next_byte();
        Serial.print(F("  Long record, length="));
        Serial.println(length);
        incr_addr(length);
      } else {
        length = b1;
        for (int i=0;i<8;i++) {
          filename[i] = read_next_byte();
        }
        filename[8]=0;
        uint8_t id = read_next_byte();
        if (rec_type < 16) Serial.print("0x0");
        else Serial.print("0x");
        Serial.print(rec_type, 16);
        switch (rec_type & 0x7f) {
          case 0x01:
            Serial.print(" [Data]  ");
            break;
          case 0x02:
            Serial.print(" [Diary] ");
            break;
          case 0x03:
            Serial.print(" [OPL]   ");
            break;
          case 0x04:
            Serial.print(" [COMMS] ");
            break;
          case 0x05:
            Serial.print(" [SS]    ");
            break;
          case 0x06:
            Serial.print(" [PAGER] ");
            break;
          case 0x07:
            Serial.print(" [Notes] ");
            break;                        
          default:
            Serial.print("         ");
        }
        Serial.print(filename);
        Serial.print(" 0x");
        if (id<16) Serial.print("0");
        Serial.print(id, 16);
        Serial.print(" ");
        
        if (rec_type < 0x80) Serial.print(" Y ");
        else Serial.print("   ");
        Serial.print(length);
        Serial.println();
      }
    }
}

int blank_check() {
  Serial.print("Blank Check ");
  int blank=(read_byte(0) == 0xff);
  for (uint32_t i=1;blank && (i<eprom_size);i++) {
    blank = (read_next_byte() == 0xff);
    if ((i % 1024) == 0) {
      Serial.print(".");
    }
  }
  Serial.println(blank ? "Yes": "No");
  return blank;
}

#ifdef ZERO_COUNT
uint32_t zero_bit_count() {
  Serial.print("Zero bit count");
  uint32_t zcount=0;
  uint32_t i = 0;
  byte data = read_byte(i);
  while(i<eprom_size) {
    if ((data & 1) == 0) zcount++;
    if ((data & 2) == 0) zcount++;
    if ((data & 4) == 0) zcount++;
    if ((data & 8) == 0) zcount++;
    if ((data & 16) == 0) zcount++;
    if ((data & 32) == 0) zcount++;
    if ((data & 64) == 0) zcount++;
    if ((data & 128) == 0) zcount++;
    if ((i % 1024) == 0) {
      Serial.print(".");
    }
    i++;
    data = read_next_byte();
  }
  Serial.print(zcount); Serial.println(" zero bits");
  return zcount;
}
#endif

static int duty=900;
boolean vpp = false;

void vpp_on() {
  timer1.pwm(PWMA, duty);
  vpp = true;
}

void vpp_off() {
  timer1.disablePwm(PWMA);
  digitalWrite(PWMA, LOW);
  vpp = false;
}

float read_vpp() {
  analogReference(INTERNAL); // 1.1V
  float sum=0.0;
  for (int i=0;i<10;i++) {
    sum += analogRead(VPP_IN);
  }
  return (sum/276);
}

void vpp_calibrate(float target) {
  vpp_on();
  Serial.println(read_vpp());
  vpp_off();
}

/*
 * Program a single byte of the EPROM
 * Address bus must be set to correct address
 * Address will be programmed with 'data' and verified. 
 * Returns true on sucesss, false on failure
 * Minimum time 12ms
 * Max Time 120ms
 */
boolean program_eprom_byte(uint8_t data) {
  int count=0;
  uint8_t d;     
  digitalWrite(SOE_B, LOW);
  digitalWrite(SS_B, LOW); // Slot enable
  d = read_data_bus();
  digitalWrite(SS_B, HIGH);
  //Serial.print("Current=");Serial.print(d,16);
  //Serial.print(" required=");Serial.println(data,16);
  while(d != data) {
    digitalWrite(SOE_B, HIGH); 
    write_data_bus(data);
    vpp_on();
    delay(1); // Wait for Vpp to stabilise
    digitalWrite(SS_B, LOW); // Slot enable
    delay(10);
    digitalWrite(SS_B, HIGH); 
    vpp_off();
    // Now verify the byte
    delay(1); // wait for Vpp to drop
    set_read();
    digitalWrite(SOE_B, LOW);
    digitalWrite(SS_B, LOW); // Slot enable
    d = read_data_bus();
    digitalWrite(SS_B, HIGH); 
    if (count++ >=10) break;
  }
  //Serial.print("After prog. Current=");Serial.print(d,16);
  //Serial.print(" required=");Serial.print(data,16);
  //Serial.print(" count=");Serial.println(count);
  return (data == d);
}

boolean program_byte(uint8_t data) {
  if (pak_header.flags & 2) {
    return program_eprom_byte(data);
  } else {
    print_error("Can only write to EPROM PAKs", Serial);
    return false;
  }
}

/*
 * Print a number as hex, padded with 0s to fill at least 'digits' characters
 */
void print_hex(Stream &out, uint32_t data, byte digits)
{
  uint32_t pad = (uint32_t)0xf<<((digits-1)*4);
  //Serial.print(digits); Serial.print(",pad=");  Serial.println(pad, 16);
  while ((pad > 0) && (data & pad) == 0)
  {
    out.print('0');
    pad >>=4;
  }
  if (data>0) out.print(data, 16);
}

/*
 * Dump the memory as SRecords
 */
void dump_srec(Stream &out)
{
  out.println();
  #define BYTES_PER_LINE 32
  int addr_len = 2;
  char data_record_type = '1';
  //char end_record_type = '9';
  if (eprom_size > 0xffffff)
  {
    addr_len = 4;
    data_record_type = '3';
    //end_record_type = '7';
  }
  else if (eprom_size > 0xffff)
  {
    addr_len = 3;
    data_record_type = '2';
    //end_record_type = '8';
  }
  uint8_t byte_count = addr_len + BYTES_PER_LINE + 1;
  uint8_t checksum;
  byte data = read_byte(0);
  for (uint32_t i=0;i<1024;i+=BYTES_PER_LINE)
  {
    out.print('S');
    out.print(data_record_type);
    print_hex(out, byte_count, 2);
    checksum = byte_count;
    // print address bytes
    print_hex(out, i, addr_len*2);
    uint32_t tmp = i;
    while(tmp>0)
    {
      checksum += tmp & 0xff;
      tmp >>=8;
    }
    for (uint8_t j=0;j<BYTES_PER_LINE;j++)
    {
      print_hex(out, data, 2);
      checksum += data;
      data = read_next_byte();
    }
    checksum = ~ checksum;
    print_hex(out, checksum, 2);
    out.println();
  }
}
void print_error(const char *msg, Stream &out)
{
  out.print("ERROR:");
  out.println(msg);
}



/*
 * Read a line from 'in'
 * Line is terminated by CR or LF
 * Up to 80 chars read
 * Return is a pointer to static buffer containing the line
 */
char *readline(Stream &inout, bool echo)
{
#define MAX_INPUT 80
static char buffer[MAX_INPUT+1];
  uint8_t idx = 0;
  char c;
  do
  {
    while (inout.available() == 0) ; // wait for a char this will block
    c = inout.read();
    if (c=='\r' || c=='\n' || idx>=MAX_INPUT) break;
    if (echo)
    {
      inout.print(c);
    }
    buffer[idx++] = c;
  }
  while (true); 
  buffer[idx] = 0;
  return buffer;
}

char *readline(Stream &in)
{
  return readline(in, false);
}

/* 
 * Process a single Srecord only need to handle S1/2/3/ records
 S0 - ignore
 S1, S2, S3 - program flash
 S4, S5, S6 - ignore
 S7, S8, S9 - ignore

S	Type	ByteCount	Address	Data	Checksum
1 1     2         4,6,8   ?     2

*/
void process_srec(Stream &in, Stream &out)
{
  out.print("S");
  // read remainder of the line (S has already been read)
  char *p = readline(in, true); // will return a static buffer to the line
  // will limit each line to 32 bytes of data
#define MAX_DATA_LEN 32
  char tmp[9]; // temp buffer for atol conversion
  byte data[MAX_DATA_LEN]; 
  if (strlen(p) < 9)
  {
    print_error("S-record too short", out);
    in_srec = false;
    return;
  }
  int addr_len = 0;
  int type = *p-'0';
  switch (type)
  {
  case 1:
    in_srec = true;
    addr_len = 4;
    break;
  case 2:
    in_srec = true;
    addr_len = 6;
    break;
  case 3:
    in_srec = true;
    addr_len = 8;
    break;
  case 0:
    in_srec = true;
    break;
  case 4:	// reserved
    break;
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
    in_srec = false;
    break;
  default:
    // error
    in_srec = false;
    print_error("Invalid s-record", out);
    break;
  }
  if (addr_len == 0) return;
  p++; // points to start of byte count
  // get byte count
  strncpy(tmp, p, 2);
  tmp[2] = '\0';
  int byte_count = (int)strtol(tmp, NULL, 16);
  if (byte_count <3) 
  {
    print_error("Invalid s-record (byte count too small)", out);
    in_srec = false;
    return;
  }
  //out.print("byte_count="); out.println(byte_count);
  p+=2; // points to address
  // get address
  strncpy(tmp, p, addr_len);
  tmp[addr_len] = '\0';
  long address = strtol(tmp, NULL, 16);
  uint8_t my_check = byte_count;
  uint32_t t = address;
  while(t>0)
  {
    my_check += t & 0xff;
    t >>=8;
  }
  p+=addr_len;
  byte_count -= addr_len/2; // skip address
  byte_count -= 1; // don't count checksum
  for (int i=0;i<byte_count;i++)
  {
    strncpy(tmp, p, 2);
    tmp[2] = '\0';
    p+=2;
    data[i] = (int)strtol(tmp, NULL, 16);
    my_check += data[i];
  }
  my_check = ~ my_check;
  strncpy(tmp, p, 2);
  tmp[2] = '\0';
  uint8_t checksum = (uint8_t)strtol(tmp, NULL, 16);
  if (my_check != checksum)
  {
    print_error("Checksum mismatch", out);
    in_srec = false;
    return;
  }
  // OK, record is valid, lets program the device
  set_address(address);
  for (int i=0;i<byte_count;i++)
  {
    if (!program_byte(data[i])) {
      out.print("-Failed@");
      print_hex(out, address+i, 4);
      out.println();
      in_srec = false;
      return;
    }
    incr_addr();
  }
  out.println("-OK");
}

void set_pak_size() {
  if ((pak_header.flags & 1) == 0) {
    Serial.print("flags are valid 0x");
    Serial.println(pak_header.flags, 16);
    if (pak_header.size > 32) eprom_size = (uint32_t)0;
    else eprom_size = pak_header.size*(uint32_t)8192;
  } else {
    Serial.print("flags are invalid 0x");
    Serial.println(pak_header.flags, 16);
    eprom_size = (uint32_t)0;
  } 
  Serial.print("eprom_size=");
  Serial.println(eprom_size);
}

void read_header() {
  uint8_t *ptr = (uint8_t *)&(pak_header.flags);
  for (int i=0;i<10;i++,ptr++) {
      *ptr = read_byte(i);
  }
  set_pak_size();
}

/* 
  prompt for header values
*/
void prompt_for_header()
{
    while (Serial.read() != -1);	// flush serial
    Serial.print("Enter PAK flags (0x7a=linear datapak, 0x7e=paged datapak, 0x7c=rampak, 0x26=flashpak): ");
    pak_header.flags = (uint8_t)strtol(readline(Serial, true), NULL, 0);
    Serial.println();
    while (Serial.read() != -1);	// flush serial
    Serial.print("Enter PAK size (8k blocks): ");
    pak_header.size = (uint8_t)strtol(readline(Serial, true), NULL, 0);
    Serial.println();
    set_pak_size();
}

boolean header_is_valid()
{
	// allow for up to 512k PAKS
	if (pak_header.size == 0 || pak_header.size > 64 || (pak_header.flags & 0x1))
	{
		Serial.println("PAK header invalid");
		return false;
	}
	return true;
}



bool enabled = false; // needs to be enabled to program

bool check_enabled() {
  if (!enabled) {
    Serial.println("Not enabled");
  }
  return enabled;
}
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("PSION II PAK reader/programmer");
  set_read();
  bus_idle();
  pinMode(SMR, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(SOE_B, OUTPUT);
  pinMode(SS_B, OUTPUT);
  pinMode(SPGM_B, OUTPUT);
  
  timer1.initialize(20);
  pinModeFast(PWMA, OUTPUT);
  digitalWriteFast(PWMA, LOW);
  read_header();
  print_pak_header(&pak_header);
}

void loop() {
  int ch = -1;

  if (!in_srec) {
    Serial.println();
    Serial.print("Programming ");
    Serial.print(enabled?"enabled":"disabled");
    Serial.print(" VPP ");
    Serial.print(vpp?"ON":"OFF");
    Serial.print(" VPP Duty ");
    Serial.print(duty);
    Serial.println(". ? for help");
  }
  Serial.print(": ");

  while (ch == -1) ch = Serial.read();

  switch(ch) {
    case '?':
      Serial.println();
      Serial.println("b : Blank check");
      Serial.println("d : Dir");
      Serial.println("e : Enable");
	    Serial.println("h : Define PAK header in RAM");
 	    Serial.println("o : Show current PAK header in RAM");
#ifdef TEST_PROG
    	Serial.println("p : Program byte *");
#endif
  	  Serial.println("r : Read header from PAK");
   	  Serial.println("S : Start of S-record");
      Serial.println("u : Dump PAK contents");
  	  Serial.println("v : VPP toggle *");
  	  Serial.println("w : Write header to PAK *");
#ifdef ZERO_COUNT
  	  Serial.println("z - Zero count");
#endif
  	  Serial.println("- : Decrease VPP *");
  	  Serial.println("+ : Increase VPP *");
      Serial.println("Items marked with * need to be enabled");
      break;
    case 'e':
      enabled = true;
      break;
    case 'r':
      read_header();
      Serial.println("PAK header read into RAM");
      // fall-through
    case 'o':
      print_pak_header(&pak_header);
      break;
    case 'h':
      print_pak_header(&pak_header);
      prompt_for_header();
      break;
    case 'w':
      Serial.println("TODO");
      break;
    case 'd':
      if (header_is_valid()) read_dir();
      break;
    case 'b':
      if (header_is_valid()) blank_check();
      break;
#ifdef TEST_PROG
    case 'p':
      if (check_enabled()) {
        Serial.println("Programming");
        uint8_t d = read_byte(0x17);
        Serial.print("Current Address=0x");Serial.println(current_address, 16);
        Serial.print("Data=0x");Serial.println(d,16);
        Serial.print(">");
        Serial.println(program_byte(0x20));
      }
#endif
      break;
    case 'S':
      if (check_enabled() && header_is_valid()) {
        process_srec(Serial, Serial);
      } else {
        while (ch == -1) ch = Serial.read();
      }
      break;
    case 'u':
      if (header_is_valid()) dump_srec(Serial);
      break;
#ifdef ZERO_COUNT
    case 'z':
      get_valid_header();
      zero_bit_count();
      break;
#endif
    case 'v':
      if (check_enabled()) {
        if (vpp) vpp_off();
        else vpp_on();
      }
      break;
    case '-':
      if (check_enabled()) {
        duty -= 10;
        if (duty<0) duty=0;
        Serial.print("\r: VPP duty ");
        Serial.println(duty);
        timer1.setPwmDuty(PWMA, duty);
      }
      break;
    case '+':
      if (check_enabled()) {
        duty += 10;
        if (duty>1023) duty=1023;
        Serial.print("\r: VPP duty ");
        Serial.println(duty);
        timer1.setPwmDuty(PWMA, duty);
      }
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.println("Invalid");
      break;
  }
}

// vim: ai sw=2 ts=2 expandtab
