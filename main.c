// includes
#include <stdio.h>
#include <dos.h>
#include <mem.h>
#include <alloc.h>
#include <process.h>
#include <conio.h>


// CPU mode defines
#define CPU_REAL	0
#define CPU_V86		1

// controller type
#define _UHCI 0x00
#define _OHCI 0x10
#define _EHCI 0x20

// UHCI flags
#define DEVICE				1
#define CONFIGURATION		2
#define STRING				3
#define	INTERFACE			4
#define ENDPOINT			5

#define UHCI_TIMEOUT	10000
#define UHCI_ERROR_MASK (1<<1)

//  UHCI commands
#define GET_STATUS			0
#define	CLEAR_FEATURE		1
#define	SET_FEATURE			3
#define	SET_ADDRESS			5
#define GET_DESCRIPTOR		6
#define SET_DESCRIPTOR		7
#define	GET_CONFIGURATION 	8
#define SET_CONFIGURATION 	9
#define GET_INTERFACE		10
#define SET_INTERFACE		11
#define SYNCH_FRAME			12

#define SET_REPORT 0x09
// UHCO registers
#define USBCMD 			0x00
#define USBSTS 			0x02
#define USBINTR	 		0x04
#define FRNUM			0x06
#define FLBASEADD 		0x08
#define SOFTMOD			0x0C
#define PORTSC0	  		0x10
#define PORTSC1			0x12

// HUB DEFINES

#define C_HUB_LOCAL_POWER 0
#define C_HUB_OVER_CURRENT 1

#define HUB_CLASS_DESCRIPTOR 0x2900

#define PORT_CONNECTION 0
#define GET_HUB_DESCRIPTOR  6
#define PORT_ENABLE 		1
#define PORT_SUSPEND 		2
#define PORT_RESET 			4
#define PORT_POWER			8
#define C_PORT_CONNECTION 0x10


#define DATA0 (0L<<19)
#define DATA1 (1L<<19)
#define ACTIVE (0x80L<<16)

#define DEVICE_TO_HOST (1L<<7)
#define HOST_TO_DEVICE (0L<<7)

#define SETUP_INTERFACE 0x01
#define SETUP_DEVICE    0x00

#define STANDARD_QUERY	(0<<5)
#define INTERFACE_QUERY (1<<5)
#define SPECIFIC_QUERY  (2<<5)

#define SETUP 0x2D
#define IN 		0x69
#define OUT   0xE1
#define TRIPLE_ERROR (3L<<27)

#define PORT_EMPTY					0x00
#define LOW_SPEED_CONNECTED 0x01
#define HIGH_SPEED_CONNECTED 0x02

#define bool	char
#define true	1
#define	false	0

// logfile handle
FILE * f_log = NULL;

// settings
int	bDPMI = 0;
int bVCPI = 0;
int b8086 = 0;
int b80386 = 0;
int bProtectedMode = 0;
int	bPCI = 0;


// define UHCI controller struct
typedef struct UHCIController {
    unsigned char USBDevNum;		// device number (function)
    unsigned char USBBusNum;
    unsigned char USBVendorID;		// vendor id
    unsigned int  USBDeviceID;		// device id
    unsigned int  USBBaseAddr;
    unsigned char USBIrqNum;
    void far * 	  pFrameList;

} __UHCIController;
//---------------------------------------------------------------
// UHCI structures
// ----------------------------------------------------------
typedef struct UHCI_QH {
    unsigned long horz;
    unsigned long vert;
    unsigned char reserved[16];
} __UHCI_QH;
// ----------------------------------------------------------
typedef struct UHCI_TD
{
    unsigned long link_ptr;
    unsigned long stats;
    unsigned long info;
    unsigned long buffer;
    unsigned char resv[16];
} __UHCI_TD;
// ----------------------------------------------------------
typedef struct uhciSetupPacket {
    unsigned char  requestType;
    unsigned char	 requestID;
    unsigned int   wValue;
    unsigned int	 wIndex;
    unsigned int   wLength;
} __uhciSetupPacket;

typedef struct uhciHubStatus {
    unsigned int statusBits;
    unsigned int changeBits;
} __uhciHubStatus;
typedef struct uhciHubDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned char bNbrPorts;
    unsigned int  wHubCharacteristics;
    unsigned char bPwrOn2PwrGood;
    unsigned char bHubContrCurrent;
    unsigned char bDeviceRemovable;
    unsigned char bPortPwrCtrlMask;
} __uhciHubDescriptor;

typedef struct uhciDescriptor {
    unsigned char bLength;
    unsigned char bDescriptorType;
    unsigned int  bcdUSB;
    unsigned char bDeviceClass;
    unsigned char nDeviceSubClass;
    unsigned char bDeviceProtocol;
    unsigned char bMaxPacketSize;
    unsigned int  idVendor;
    unsigned int  idProduct;
    unsigned int  bcdDevice;
    unsigned char iManufacturer;
    unsigned char iProduct;
    unsigned char iSerialNumber;
    unsigned char iConfigs;
};
//---------------------------------------------------------------
// list of found controllers
UHCIController UHCIControllers[32];
unsigned char nUHCIFound = 0;	// amount of UHCI controllers found

// function prototypes

// dump content of TD memory area
void dumpTD(void far * td, bool bRecursive = true);
// dump content of QH memory area
void dumpQH(void far * qh, bool bRecursive = true);
// dump content of framelist
void dumpFrameList(void far * list, bool bRecursive = true);
// write something to LOG
void log(const char * msg);
// write value to PCI registry
void pci_write(unsigned char bus,
    unsigned char dev,
    unsigned char func,
    unsigned char ofs,
    unsigned char bSize,
    unsigned long data);
void uhciTakeFromBIOS(int ic);

//----------------------------------------------------------------
void uhciInitStructure(UHCIController * hc);
unsigned int uhciGetStatusRegister(int ic);
unsigned int uhciGetCurrentFrame(int ic);
int  uhciIsDeviceConnected(int ic, unsigned int port);
void uhciRun(int ci);
void uhciReset(int ci); // reset controller specified by ci
void uhciGetInitData(int ci);
void uhciTakeFromBIOS(int ic);
void uhciSetFrameListItem(void far * frameList, int index, void far * objectPtr, unsigned char mask = 0x01);
void uhcpDumpFrameList(int ic, bool bRecursive);
void uhciSetAddress(int ic, int address);
unsigned int uhciStatus(int ic);
void uhciDisablePort(int ic, unsigned int port);
void uhciEnablePort(int ic, unsigned int port);
void uhciResetFrameNumber(int ic);
//----------------------------------------------------------------
int is80386(void);
int is8086(void);
int isPCI(void);
void waitForKey();
//----------------------------------------------------------------
unsigned long _faddr(void far * ptr);
void align_free(void *ptr);
void far * align_malloc(size_t size, size_t alignment);
//----------------------------------------------------------------

void pci_write(unsigned char bus,
    unsigned char dev,
    unsigned char func,
    unsigned char ofs,
    unsigned char bSize,
    unsigned long data) {

    asm{
        mov bh, bus
        mov bl, dev
        mov cl, func
        mov ch, ofs
        mov dl, bSize
        mov eax, data

        push eax
        push dx
        mov eax, 0x8000
        mov al, bh
        shl eax, 5
        or al, bl
        shl eax, 3
        or al, cl
        shl eax, 6
        push cx
        shr ch,2
        or al, ch
        pop cx
        shl eax,2
        mov dx, 0x0CF8
        out dx, eax
        mov dl, ch
        and dx, 0x0003
        add dx, 0x0CF8
        in eax, dx
        pop cx
        shl cl,3
        mov ebx, 0x0FFFFFFFF
        shl ebx,cl
        and eax,ebx
        pop ebx
        or eax, eax
        out dx, eax
    }

}
// -------------------------------------------------------------------
void log(const char * msg) {
    f_log = fopen("out.txt", "a+");
    if (!f_log)
        return;
    struct time t;

    gettime(&t);

    fprintf(f_log, "%02d:%02d:%02d.%02d %s\n", t.ti_hour,
        t.ti_min,
        t.ti_sec,
        t.ti_hund,
        msg);

    fclose(f_log);
}
//-------------------------------------------------------------------
void uhciReset(int ci) {
    if (ci >= nUHCIFound)
        return;

    printf(" -------   Resetting UHCI (%.02X) -------- \n", ci);
    outpw(UHCIControllers[ci].USBBaseAddr + USBCMD, 0x04);
    delay(100);
    outpw(UHCIControllers[ci].USBBaseAddr + USBCMD, 0x00);
    delay(100);
    outpw(UHCIControllers[ci].USBBaseAddr + USBCMD, 0x02);
    do {
    } while (inpw(UHCIControllers[ci].USBBaseAddr + USBCMD) & 0x02);

}
//-------------------------------------------------------------------
void uhciRun(int ic) {
    outpw(UHCIControllers[ic].USBBaseAddr + USBCMD, 0x01);
    delay(30);
}
//-------------------------------------------------------------------
unsigned int uhciGetStatusRegister(int ic) {
    unsigned int status = 0;
    status = inpw(UHCIControllers[ic].USBBaseAddr + USBSTS);
    return status;
}
//-------------------------------------------------------------------
unsigned int uhciGetCommandRegister(int ic) {
    unsigned int status = 0;
    status = inpw(UHCIControllers[ic].USBBaseAddr + USBCMD);
    return status;
}

//-------------------------------------------------------------------
unsigned int uhciGetCurrentFrame(int ic) {
    unsigned int frame = 0;
    frame = inpw(UHCIControllers[ic].USBBaseAddr + FRNUM);
    return frame;
}
//-------------------------------------------------------------------
void uhciResetFrameNumber(int ic) {
    unsigned int cmd = 0;
    cmd = inpw(UHCIControllers[ic].USBBaseAddr + USBCMD);
    cmd &= 0xFE;
    outpw(UHCIControllers[ic].USBBaseAddr + USBCMD, cmd);

    delay(30);

    outpw(UHCIControllers[ic].USBBaseAddr + FRNUM, 0x00);

    delay(30);

    cmd |= 0x01;
    outpw(UHCIControllers[ic].USBBaseAddr + USBCMD, cmd);

    delay(30);
}
//-------------------------------------------------------------------
void uhciInitStructure(UHCIController * hc) {
    if (!hc)
        return;

    memset(hc, 0, sizeof(UHCIController));
}
//-------------------------------------------------------------------
void uhciGetInitData(int ci) {
    if (ci >= nUHCIFound)
        return;

    unsigned char __BH;
    unsigned char __BL;

    __BH = UHCIControllers[ci].USBBusNum;
    __BL = UHCIControllers[ci].USBDevNum;

    // VendorID
    _BH = __BH;
    _BL = __BL;
    _ECX = 0x000c0300;
    _SI = 0;
    _AX = 0xB109;
    _DI = 0;
    geninterrupt(0x1A);
    UHCIControllers[ci].USBVendorID = _CX;

    // DeviceID
    _AX = 0xB109;
    _DI = 2;
    _BH = __BH;
    _BL = __BL;
    _ECX = 0x000c0300;
    _SI = 0;
    geninterrupt(0x1A);
    UHCIControllers[ci].USBDeviceID = _CX;

    // BASE Address
    _AX = 0xB10A;
    _DI = 0x20;
    _SI = 0;
    _ECX = 0x000c0300;
    _BH = __BH;
    _BL = __BL;
    geninterrupt(0x1A);
    UHCIControllers[ci].USBBaseAddr = _CX;
    UHCIControllers[ci].USBBaseAddr &= 0xFFE0;

    // IRQ Number
    _AX = 0xB108;
    _DI = 0x3C;
    _ECX = 0x000c0300;
    _SI = 0;
    _BH = __BH;
    _BL = __BL;
    geninterrupt(0x1A);
    UHCIControllers[ci].USBIrqNum = _CL;
}
//--------------------------------------------------------------------
void uhciAddController(unsigned char bus, unsigned char device) {
    UHCIController uh;
    uhciInitStructure(&uh);

    uh.USBBusNum = bus;
    uh.USBDevNum = device;

    UHCIControllers[nUHCIFound] = uh;
    nUHCIFound++;

    uhciGetInitData(nUHCIFound - 1);
}

// -------------------- MEMORY ALLOCATION PART --------------
unsigned int totalAllocated = 0;
// ----------------------------------------------------------
unsigned long _faddr(void far * ptr) {
    return ((unsigned long)FP_SEG(ptr) << 4) + FP_OFF(ptr);
}
// ----------------------------------------------------------
void align_free(void *ptr) {
    if (ptr) {
        farfree(*((void far**)ptr - 1));
    }
}
// ----------------------------------------------------------
void far * align_malloc(size_t size, size_t alignment) {
    void far * pa, far *ptr;
    int nSize = (size + alignment - 1) + sizeof(void far *);
    //	printf("About to allocate %i aligned bytes (free %lu)\n", nSize, farcoreleft());
    pa = farmalloc(nSize);
    if (!pa) {
        printf("Error allocating memory (%u)!!!!\n", totalAllocated);
        exit(-1);
        return NULL;
    }

    totalAllocated += nSize;

    unsigned long linear = (((unsigned long)FP_SEG(pa)) << 4) + FP_OFF(pa);
    unsigned long new_linear = ((linear + sizeof(void far *) + alignment - 1)) / alignment * alignment;

    unsigned int seg = new_linear >> 4;
    unsigned int off = new_linear - (seg << 4);

    ptr = MK_FP(seg, off);

    *((void far **)ptr - 1) = pa;

    return ptr;
}
// ----------------------------------------------------------
void uhciInitFramesList(int ic) {
    if (ic >= nUHCIFound)
        return;

    void far * frameListPtr = align_malloc(4096, 4096);
    UHCI_QH far * qh = (UHCI_QH far *)align_malloc(32, 16);

    _fmemset(qh, 0, sizeof(UHCI_QH));
    qh->horz = 1;
    qh->vert = 1;

    for (int i = 0; i < 1024; i++) {
        uhciSetFrameListItem(frameListPtr, i, (void far*)qh, 2
        );
    }

    UHCIControllers[ic].pFrameList = frameListPtr;
}
// ----------------------------------------------------------
void uhciSetFrameListAddress(int ic) {
    unsigned long linearResponse;
    unsigned int _reg = UHCIControllers[ic].USBBaseAddr + FLBASEADD;
    unsigned long newAddress = _faddr(UHCIControllers[ic].pFrameList);
    printf("Setting buffer address to: %.04lX\n", newAddress);
    _EAX = newAddress;
    _DX = _reg;
    asm{
        out dx, eax;
    }
}
//-----------------------------------------------------------
unsigned long uhciGetFrameListAddress(int ic) {
    unsigned long newAddress;
    unsigned int _reg = UHCIControllers[ic].USBBaseAddr + FLBASEADD;
    _DX = _reg;
    asm{
        in eax, dx;
    }
    newAddress = _EAX;
    return newAddress;
}
// ----------------------------------------------------------
void uhciDisablePort(int ic, unsigned int port) {
    unsigned int ps = inpw(UHCIControllers[ic].USBBaseAddr + port);
    ps &= 0xFFFB;
    outpw(UHCIControllers[ic].USBBaseAddr + port, ps);
}
// ----------------------------------------------------------
void uhciEnablePort(int ic, unsigned int port) {
    unsigned int ps = inpw(UHCIControllers[ic].USBBaseAddr + port);
    ps |= 0x04;
    outpw(UHCIControllers[ic].USBBaseAddr + port, ps);
}
// ----------------------------------------------------------
void uhciSetFrameListItem(void far * frameList, int index, void far * objectPtr, unsigned char mask) {
    unsigned long linear = ((unsigned long)FP_SEG(objectPtr) << 4) + FP_OFF(objectPtr);
    linear = linear & 0xFFFFFFF0;
    linear = linear | mask;

    _fmemcpy(&(((unsigned char far *)frameList)[index * 4]),
        (unsigned char far*)&linear, 4);
}
// -----------------------------------------------------------
unsigned int uhciStatus(int ic) {
    return inpw(UHCIControllers[ic].USBBaseAddr + USBSTS);
}
// -----------------------------------------------------------
void uhciSetAddress(int ic, int address) {

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * stop;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x00;
    info->requestID = SET_ADDRESS;
    info->wValue = address;
    info->wIndex = 0;
    info->wLength = 0;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 8; j++) {
        printf("%02X ", ptrD[j]);
    }
    td->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP);
    td->buffer = _faddr(info);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = (0x7FFL << 21) | (DATA1 | IN); // ((0x7FFL<<21) | DATA1 | OUT);
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);
    unsigned long currentStats = stop->stats;
    printf("Status: %.04lX\n", currentStats);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\n");
            //	dumpTD(td);
        }

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));


    delay(50);
    if (count == UHCI_TIMEOUT) {
        printf("uhciSetAddress: Timeout while setting an address\n");
        return;
    }
    else {
        printf("uhciSetAddress: Operation complete\n");
    }
}
// -----------------------------------------------------------
void uhciDumpDescriptor(uhciDescriptor * d) {
    printf("---------- Descriptor -------\n");
    printf("Length:    %.02X\n", d->bLength);
    printf("Type:      %.02X\n", d->bDescriptorType);
    printf("USB:       %.04X\n", d->bcdUSB);
    printf("Class:     %.02X\n", d->bDeviceClass);
    printf("SubClass:  %.02X\n", d->nDeviceSubClass);
    printf("Proto:     %.02X\n", d->bDeviceProtocol);
    printf("MaxPSize:  %.02X\n", d->bMaxPacketSize);
    printf("Vendor:    %.04X\n", d->idVendor);
    printf("Product:   %.04X\n", d->idProduct);
    printf("Device:    %.04X\n", d->bcdDevice);
    printf("Mnfct:     %.02X\n", d->iManufacturer);
    printf("Product:   %.02X\n", d->iProduct);
    printf("SerNumber: %.02X\n", d->iSerialNumber);
    printf("Mnfct:     %.02X\n", d->iConfigs);
    printf("-----------------------------\n");
}
// -----------------------------------------------------------
uhciDescriptor * uhciGetDescriptor(int ic, unsigned char address, uhciDescriptor * d) {

    if (address);

    printf("Getting descriptor for device: %d\n", address);
    memset(d, 0, sizeof(uhciDescriptor));

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * td2;
    UHCI_TD far * td3;

    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(32, 16);
    memset(dd, 0, sizeof(uhciDescriptor));

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);
    td2 = (UHCI_TD far *)align_malloc(32, 16);
    td3 = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    _fmemset(qh, 0, sizeof(UHCI_QH));
    _fmemset(td, 0, sizeof(UHCI_TD));
    _fmemset(td1, 0, sizeof(UHCI_TD));
    _fmemset(td2, 0, sizeof(UHCI_TD));
    _fmemset(td3, 0, sizeof(UHCI_TD));
    _fmemset(stop, 0, sizeof(UHCI_TD));

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x80;
    info->requestID = GET_DESCRIPTOR;
    info->wValue = ((unsigned int)DEVICE << 8);
    info->wIndex = 0;
    info->wLength = 18;

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(td2) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((7L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    td2->link_ptr = (_faddr(td3) & 0xFFFFFFF0) | 0x04;
    td2->stats = (TRIPLE_ERROR | ACTIVE);
    td2->info = ((7L << 21) | DATA0 | IN | ((address & 0x7F) << 8));
    td2->buffer = _faddr(dd + 8);

    td3->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td3->stats = (TRIPLE_ERROR | ACTIVE);
    td3->info = ((1L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td3->buffer = _faddr(dd + 16);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA0 | OUT | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;
    printf("Last controller status: %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\r");
            //	dumpTD(td);
        }

        if (!(td1->stats & ACTIVE))
            printf("TD1:done\r");

        if (!(td2->stats & ACTIVE))
            printf("TD2:done\r");

        if (!(td3->stats & ACTIVE))
            printf("TD3:done\r");

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\r");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        //		if (controllerStatus & UHCI_ERROR_MASK) {
        //			printf("USBCMD:%.02X\n",cmdRegister);
        //			printf("Error in transaction\n");
        //			return NULL;
        //		}

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    if (count == UHCI_TIMEOUT) {
        printf("uhciGetDescriptor: Timeout while getting device descriptor\n");
        return NULL;
    }
    else {
        printf("uhciGetDescriptor: Operation complete\n");
    }

    _fmemcpy(d, dd, sizeof(uhciDescriptor));

    printf("Dump: ");
    for (int q = 0; q < 18; q++) {
        printf("%.02X ", dd[q]);
    }

    printf("\n");
    return d;
}
// ----------------------------------------------------------
void uhciDumpHubDescriptor(uhciHubDescriptor * d) {
    printf("------------- HUB Descriptor -------------- \n");
    printf("bLength:             %d\n", d->bLength);
    printf("bDescriptorType:     %02X\n", d->bDescriptorType);
    printf("bNbrPorts:           %d\n", d->bNbrPorts);
    printf("PowerUp Time:        %02X(%d)\n", d->bPwrOn2PwrGood, d->bPwrOn2PwrGood);
    printf("------------------------------------------- \n");
}
// -----------------------------------------------------------
uhciHubDescriptor * uhciGetHubDescriptor(int ic, unsigned char address, uhciHubDescriptor * d) {

    if (address);

    printf("Getting hub descriptor for device: %d\n", address);
    memset(d, 0, sizeof(uhciDescriptor));

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * td2;
    UHCI_TD far * td3;

    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(32, 16);
    memset(dd, 0, sizeof(uhciHubDescriptor));

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);
    td2 = (UHCI_TD far *)align_malloc(32, 16);
    td3 = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    _fmemset(qh, 0, sizeof(UHCI_QH));
    _fmemset(td, 0, sizeof(UHCI_TD));
    _fmemset(td1, 0, sizeof(UHCI_TD));
    _fmemset(td2, 0, sizeof(UHCI_TD));
    _fmemset(td3, 0, sizeof(UHCI_TD));
    _fmemset(stop, 0, sizeof(UHCI_TD));

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0xA0;
    info->requestID = GET_HUB_DESCRIPTOR;
    info->wValue = HUB_CLASS_DESCRIPTOR;
    info->wIndex = 0;
    info->wLength = 14;

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(td2) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((7L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    td2->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td2->stats = (TRIPLE_ERROR | ACTIVE);
    td2->info = ((7L << 21) | DATA0 | IN | ((address & 0x7F) << 8));
    td2->buffer = _faddr(dd + 8);

    /*	td3->link_ptr  = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
        td3->stats 	  	= (TRIPLE_ERROR | ACTIVE);
        td3->info 			= ((1L<<21) | DATA1 | IN | ((address & 0x7F) << 8));
        td3->buffer 		= _faddr(dd+16);
        */

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;
    printf("Last controller status: %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\r");
            //	dumpTD(td);
        }

        if (!(td1->stats & ACTIVE))
            printf("TD1:done\n");

        if (!(td2->stats & ACTIVE))
            printf("TD2:done\n");

        /*		if (!(td3->stats & ACTIVE))
                    printf("TD3:done\n");
            */

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        //		if (controllerStatus & UHCI_ERROR_MASK) {
        //			printf("USBCMD:%.02X\n",cmdRegister);
        //			printf("Error in transaction\n");
        //			return NULL;
        //		}

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    if (count == UHCI_TIMEOUT) {
        printf("uhciGetDescriptor: Timeout while getting device descriptor\n");
        return NULL;
    }
    else {
        printf("uhciGetHUBDescriptor: Operation complete\n");
    }

    _fmemcpy(d, dd, sizeof(uhciDescriptor));

    printf("Dump: ");
    for (int q = 0; q < 14; q++) {
        printf("%.02X ", dd[q]);
    }

    printf("\n");
    return d;
}

// -----------------------------------------------------------
uhciHubStatus * uhciGetHubStatus(int ic, unsigned char address, unsigned char port, uhciHubStatus * d) {

    if (address);

    printf("Getting hub port (%d) status for device: %d\n", port, address);
    memset(d, 0, sizeof(uhciHubStatus));

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(32, 16);
    memset(dd, 0, sizeof(uhciHubStatus));

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    _fmemset(qh, 0, sizeof(UHCI_QH));
    _fmemset(td, 0, sizeof(UHCI_TD));
    _fmemset(td1, 0, sizeof(UHCI_TD));
    _fmemset(stop, 0, sizeof(UHCI_TD));

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0xA3; // device to host other, class
    info->requestID = GET_STATUS;
    info->wValue = 0;
    info->wIndex = port;          //+
    info->wLength = 4; // word+word   //+

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((4L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA0 | OUT | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;
    printf("Last controller status: %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\r");
            //	dumpTD(td);
        }

        if (!(td1->stats & ACTIVE))
            printf("TD1:done\n");

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }


        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    if (count == UHCI_TIMEOUT) {
        printf("uhciGetDescriptor: Timeout while getting device descriptor\n");

        _fmemcpy(d, dd, sizeof(uhciHubStatus));
        printf("Dump: ");
        for (int q = 0; q < 18; q++) {
            printf("%.02X ", dd[q]);
        }

        return NULL;
    }
    else {
        printf("uhciHubPortStatus: Operation complete\n");
    }

    _fmemcpy(d, dd, sizeof(uhciHubStatus));

    printf("Dump: ");
    for (int q = 0; q < 4; q++) {
        printf("%.02X ", dd[q]);
    }

    printf("\n");
    return d;
}
// ----------------------------------------------------------

void dumpQH(void far * qh, bool bRecursive) {
    if (!qh)
        return;

    printf("-------- QH (%.08lX) ------------- \n", _faddr(qh));
    printf("Raw:");
    int i;
    for (i = 0; i < 8; i++) {
        printf("%.02X", ((unsigned char far*)qh)[i]);
    }
    printf("\n");

    unsigned long address;
    unsigned long part1;

    address = *((unsigned long far *)qh);


    if (address & 0x00000001) {
        printf("Link address:    N/A\n");
        printf("Next in chain:   N/A\n");
    }
    else {
        printf("Link address:  %.08lX\n", address & 0xFFFFFFF0);
        printf("Next in chain: %s\n", address & 0x00000002 ? "QUEUE HEADER" : "DESCRIPTOR");
    }
    printf("Reserved bits:   %.02X\n", address & 0x0000000C);
    printf("Is Last: %s\n", address & 0x00000001 ? "YES" : "NO");

    address = *((unsigned long far *)qh + 1);
    if (address & 0x00000001) {
        printf("Next element address:  N/A\n");
        printf("Next element in chain: N/A\n");
    }
    else {
        printf("Next element address: %.08lX\n", address & 0xFFFFFFF0);
        printf("Next in list:         %s\n", address & 0x00000002 ? "QUEUE HEADER" : "DESCRIPTOR");
    }

    printf("Reserved bits:        %.02X\n", address & 0x0000000C);
    printf("Is Last:              %s\n", address & 0x00000001 ? "YES" : "NO");

    if (!(address & 0x00000001) && bRecursive) {
        printf(" ------ >>>>> ---------- \n");
        unsigned int seg = (address & 0xFFFFFFF0) >> 4;

        if (address & 0x00000002) {
            waitForKey();
            dumpQH((void far*)MK_FP(seg, 0), bRecursive);
        }
        else {
            waitForKey();
            dumpTD((void far*)MK_FP(seg, 0), bRecursive);
        }
        printf(" ------ <<<<< ---------- \n");
    }
    printf(" ----------------------------- \n");
}

void dumpTD(void far * td, bool bRecursive) {
    if (!td)
        return;

    printf("-------- TD (%.08lX) ------------- \n", _faddr(td));
    printf("Raw:");
    int i;
    for (i = 0; i < 16; i++) {
        printf("%.02X", ((unsigned char far*)td)[i]);
    }
    printf("\n");

    unsigned long address;

    address = *((unsigned long far *)td);

    if (address & 0x00000001) {
        printf("Link address:    N/A\n");
        printf("Next in chain:   N/A\n");
        printf("Mode: 					 N/A\n");

    }
    else {
        printf("Link address:  %.08lX\n", address & 0xFFFFFFF0);
        printf("Next in chain: %s\n", address & 0x00000002 ? "QUEUE HEADER" : "DESCRIPTOR");
        printf("Mode: %s\n", address & 0x00000004 ? "DEPTH FIRST" : "BREADTH FIRST");
    }

    printf("Reserved bit:   %.02X\n", address & 0x00000008);
    printf("Is Last: %s\n", address & 0x00000001 ? "YES" : "NO");

    address = *((unsigned long far *)td + 1);

    printf("(SPD:%s) ", (address & (1L << 29)) ? "Enable" : "Disable");
    printf("(ERR:%.02X) ", address & ((1L << 28) + (1L << 27)));
    printf("(LS:%s) ", (address & (1L << 26)) ? "YES" : "NO");
    printf("(IOS:%s) ", (address & (1L << 25)) ? "YES" : "NO");
    printf("(IOC:%s) ", (address & (1L << 24)) ? "YES" : "NO");
    printf("(IOC:%s) ", (address & (1L << 24)) ? "YES" : "NO");
    printf("(Active:%s) ", (address & (1L << 23)) ? "YES" : "NO");
    printf("(Stalled:%s) ", (address & (1L << 22)) ? "YES" : "NO");
    printf("(DataBufError:%s) ", (address & (1L << 21)) ? "YES" : "NO");
    printf("(Babble Det:%s) ", (address & (1L << 20)) ? "YES" : "NO");
    printf("(NAK:%s) ", (address & (1L << 19)) ? "YES" : "NO");
    printf("(CRC/Timeout:%s) ", (address & (1L << 18)) ? "YES" : "NO");
    printf("(Bitstuff err:%s) \n", (address & (1L << 17)) ? "YES" : "NO");

    printf("Actual length:   %.03X\n", (address & 0x7FF));

    address = *((unsigned long far *)td + 2);

    printf("Max length:   	%.03X\n", ((address >> 21) & 0x7FF));
    printf("Data toggle:  	%s\n", (address & (1L << 19)) ? "DATA1" : "DATA0");
    printf("Endpoint:     	%.02X\n", ((address >> 15) & (0x0F)));
    printf("Device address: %.02X\n", ((address >> 8) & (0x7F)));
    printf("PID:            %.02X\n", (address & 0xFF));

    address = *((unsigned long far *)td + 3);
    printf("Buffer addres: 	%.08lX\n", address);

    address = *((unsigned long far *)td);

    if (!(address & 0x00000001) && bRecursive) {
        printf(" ------ >>>>> ---------- \n");
        unsigned int seg = (address & 0xFFFFFFF0) >> 4;

        if (address & 0x00000002) {
            waitForKey();
            dumpQH((void far*)MK_FP(seg, 0), bRecursive);
        }
        else {
            waitForKey();
            dumpTD((void far*)MK_FP(seg, 0), bRecursive);
        }
        printf(" ------ <<<<< ---------- \n");
    }
    printf(" ----------------------------- \n");
}
//------------------------------------------------------------------------------------------------
void uhcpDumpFrameList(int ic, bool bRecursive) {
    void far * list = UHCIControllers[ic].pFrameList;
    unsigned char isEmpty = 1;
    unsigned char far * __list = (unsigned char far *)list;
    for (int i = 0; i < 1024; i++) {

        unsigned long value = (((unsigned long)__list[i * 4])) +
            ((unsigned long)__list[i * 4 + 1] << 8) +
            ((unsigned long)__list[i * 4 + 2] << 16) +
            ((unsigned long)__list[i * 4 + 3] << 24);

        if (value == 0) {
            printf("Error!!!!\n");
            break;
        }

        int bEmpty = (__list[i * 4] & 0x01) ? 1 : 0;
        int bIsDescriptor = (__list[i * 4] & 0x02) ? 0 : 1;
        if (!bEmpty) {
            isEmpty = 0;
            printf("Frame %d, value:%lX address: %lX, Empty:%s, Type:%s\n",
                i, value, value & 0xFFFFFFF0, bEmpty ? "YES" : "NO",
                bIsDescriptor ? "DESCRIPTOR" : "QUEUE HEADER");

            unsigned int seg = (value & 0xFFFFFFF0) >> 4;
            if (bRecursive)
                if (bIsDescriptor) {
                    waitForKey();
                    dumpTD((void far*)MK_FP(seg, 0), bRecursive);
                }
                else {
                    waitForKey();
                    dumpQH((void far*)MK_FP(seg, 0), bRecursive);
                }
        }
    }

    if (isEmpty)
        printf("FrameList id Empty, filled with default's TD... OK\n");
}
// ----------------------------------------------------------
int uhciIsDeviceConnected(int ic, unsigned int port) {
    unsigned int status;
    status = inpw(UHCIControllers[ic].USBBaseAddr + port);

    if (!(status & 0x01))
        return PORT_EMPTY;

    if (status & (1L << 8))
        return LOW_SPEED_CONNECTED;

    return HIGH_SPEED_CONNECTED;
}
// ----------------------------------------------------------
void uhciTakeFromBIOS(int ic) {
    unsigned char USBFunctionNumber = UHCIControllers[ic].USBDevNum;
    USBFunctionNumber &= 0x07;

    pci_write(UHCIControllers[ic].USBBusNum,
        UHCIControllers[ic].USBDevNum,
        USBFunctionNumber, // fun,
        0x04,
        0x02,
        0x05);

    pci_write(UHCIControllers[ic].USBBusNum,
        UHCIControllers[ic].USBDevNum,
        USBFunctionNumber, // fun,
        0x0C0,
        0x02,
        0x8F00);
}
//-----------------------------------------------------------------------
void uhciHubPortPowerOn(int ic, unsigned char address, unsigned char port) {

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * stop;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x23; // OTHER+CLASS+HOSTTODEVICE
    info->requestID = SET_FEATURE; //+
    info->wValue = PORT_POWER;  //+
    info->wIndex = port;
    info->wLength = 0;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 8; j++) {
        printf("%02X ", ptrD[j]);
    }
    td->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = (0x7FFL << 21) | (DATA1 | IN) | ((address & 0x7F) << 8); // ((0x7FFL<<21) | DATA1 | OUT);
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);
    unsigned long currentStats = stop->stats;
    printf("Status: %.04lX\n", currentStats);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\n");
            //	dumpTD(td);
        }

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));


    delay(50);
    if (count == UHCI_TIMEOUT) {
        printf("uhciHubPortPower: Timeout while setting an address\n");
        return;
    }
    else {
        printf("uhciHubPortPower: Operation complete\n");
    }

}

//-----------------------------------------------------------
void uhciSetConfiguration(int ic, unsigned char address, unsigned char config) {

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * stop;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x00; //
    info->requestID = SET_CONFIGURATION; //+
    info->wValue = config;  //+
    info->wIndex = 0;
    info->wLength = 0;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 8; j++) {
        printf("%02X ", ptrD[j]);
    }
    td->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = (0x7FFL << 21) | (DATA1 | IN) | ((address & 0x7F) << 8); // ((0x7FFL<<21) | DATA1 | OUT);
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);
    unsigned long currentStats = stop->stats;
    printf("Status: %.04lX\n", currentStats);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\n");
            //	dumpTD(td);
        }

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));


    delay(50);
    if (count == UHCI_TIMEOUT) {
        printf("uhciSetConfiguration: Timeout while setting an address\n");
        return;
    }
    else {
        printf("uhciSetConfiguration: Operation complete\n");
    }
}
//-----------------------------------------------------------

void uhciHubClearFeature(int ic, unsigned char address, unsigned char port) {

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * stop;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x23; // OTHER+CLASS+HOSTTODEVICE
    info->requestID = CLEAR_FEATURE; //+
    info->wValue = C_PORT_CONNECTION;  //+
    info->wIndex = port;
    info->wLength = 0;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 8; j++) {
        printf("%02X ", ptrD[j]);
    }
    td->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = (0x7FFL << 21) | (DATA1 | IN) | ((address & 0x7F) << 8); // ((0x7FFL<<21) | DATA1 | OUT);
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);
    unsigned long currentStats = stop->stats;
    printf("Status: %.04lX\n", currentStats);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\n");
            //	dumpTD(td);
        }

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));


    delay(50);
    if (count == UHCI_TIMEOUT) {
        printf("uhciClearFeature: Timeout while setting an address\n");
        return;
    }
    else {
        printf("uhciClearFeature: Operation complete\n");
    }

}

//-----------------------------------------------------------
void uhciHubResetPort(int ic, unsigned char address, unsigned char port) {

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * stop;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x23; // OTHER+CLASS+HOSTTODEVICE
    info->requestID = SET_FEATURE; //+
    info->wValue = PORT_RESET;  //+
    info->wIndex = port;
    info->wLength = 0;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 8; j++) {
        printf("%02X ", ptrD[j]);
    }
    td->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = (0x7FFL << 21) | (DATA1 | IN) | ((address & 0x7F) << 8); // ((0x7FFL<<21) | DATA1 | OUT);
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);
    unsigned long currentStats = stop->stats;
    printf("Status: %.04lX\n", currentStats);

    unsigned int controllerStatus = uhciGetStatusRegister(ic);
    unsigned int cmdRegister = uhciGetCommandRegister(ic);

    int count = 0;
    printf("USBSTS:%.02X\n", controllerStatus);
    printf("USBCMD:%.02X\n", cmdRegister);

    do {
        if (!(td->stats & ACTIVE)) {
            printf("TD:done\n");
            //	dumpTD(td);
        }

        if (!(stop->stats & ACTIVE))
            printf("STOP:done\n");

        unsigned int newStatus = uhciGetStatusRegister(ic);
        unsigned int newCmd = uhciGetCommandRegister(ic);

        if (controllerStatus != newStatus) {
            controllerStatus = newStatus;
            printf("USBSTS:%.02X\n", controllerStatus);
        }

        if (cmdRegister != newCmd) {
            cmdRegister = newCmd;
            printf("USBCMD:%.02X\n", cmdRegister);
        }

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Last controller status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));


    delay(50);
    if (count == UHCI_TIMEOUT) {
        printf("uhciClearFeature: Timeout while setting an address\n");
        return;
    }
    else {
        printf("uhciClearFeature: Operation complete\n");
    }

}

//-----------------------------------------------------------
void broadcomDeleteDevice(int ic, unsigned char address,
    unsigned char b1,
    unsigned char b2,
    unsigned char b3,
    unsigned char b4,
    unsigned char b5,
    unsigned char b6

) {
    if (address);
    if (!b1 && !b2 && !b3 && !b4 && !b5 && !b6)
        return;

    printf("Broadcom delete report\n");

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * td2;
    UHCI_TD far * td3;
    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(32, 16);

    _fmemset(dd, 0, 32);

    dd[0] = 0x7E;
    dd[1] = 0x48;
    dd[2] = 0x00;
    dd[3] = 0x00;

    dd[4] = 0x06;

    dd[5] = b6;
    dd[6] = b5;
    dd[7] = b4;
    dd[8] = b3;
    dd[9] = b2;
    dd[10] = b1;

    dd[11] = 0;
    dd[12] = 0;
    dd[13] = 0;
    dd[14] = 0;
    dd[15] = 0;
    dd[16] = 0;
    dd[17] = 0;
    dd[18] = 0;
    dd[19] = 0;
    dd[20] = 0;
    dd[21] = 0;
    dd[22] = 0;
    dd[23] = 0;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);
    td2 = (UHCI_TD far *)align_malloc(32, 16);
    td3 = (UHCI_TD far *)align_malloc(32, 16);

    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x21; //INTERFACE CLASS H->D 0x80;
    info->requestID = SET_REPORT; // custom query (SETUP)
    info->wValue = 0x37E;
    info->wIndex = 0;
    info->wLength = 23;

    int j = 0;
    printf("Dump input data:");
    for (j = 0; j < 23; j++) {
        printf("%02X ", dd[j]);
    }

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(td2) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((7L << 21) | DATA1 | OUT | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    td2->link_ptr = (_faddr(td3) & 0xFFFFFFF0) | 0x04;
    td2->stats = (TRIPLE_ERROR | ACTIVE);
    td2->info = ((7L << 21) | DATA0 | OUT | ((address & 0x7F) << 8));
    td2->buffer = _faddr(dd + 8);

    td3->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td3->stats = (TRIPLE_ERROR | ACTIVE);
    td3->info = ((6L << 21) | DATA1 | OUT | ((address & 0x7F) << 8));
    td3->buffer = _faddr(dd + 16);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA0 | OUT | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;
    printf("Status (SETUP): %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    int count = 0;
    do {
        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Status (SETUP):%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    if (count == UHCI_TIMEOUT) {
        printf("broadcom delete device: Timeout\n");
        return;
    }
    else {
        printf("broadcom delete device: Operation complete\n");
    }

    // ----------------------------------------------------------------

/*	printf("Dump: ");
    for (int q=0;q<4;q++) {
        printf("%.02X ", dd[q]);
    }
    */
    printf("\n");
    return;
}
//-----------------------------------------------------------

void broadcomSetReport(int ic, unsigned char address) {
    if (address);

    printf("Broadcom SET report\n");

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * td2;
    UHCI_TD far * td3;
    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(32, 16);

    // hardcoded from USB dump

    _fmemset(dd, 0, 32);

    dd[0] = 0x7F;
    dd[1] = 0x20;
    dd[2] = 0x00;
    dd[3] = 0x00;

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);

    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0x21; //INTERFACE CLASS H->D 0x80;
    info->requestID = SET_REPORT; // custom query (SETUP)
    info->wValue = 0x37F;
    info->wIndex = 0;
    info->wLength = 4;

    unsigned char far * ptrD = (unsigned char far *)info;
    int j = 0;
    printf("Dump data:");
    for (j = 0; j < 4; j++) {
        printf("%02X ", ptrD[j]);
    }

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((3L << 21) | DATA1 | OUT | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA0 | OUT | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;
    printf("Status (SETUP): %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    int count = 0;
    do {
        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Status (SETUP):%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    if (count == UHCI_TIMEOUT) {
        printf("broadcom set report: Timeout\n");
        return;
    }
    else {
        printf("broadcomSetReport: Operation complete\n");
    }

    // ----------------------------------------------------------------

    printf("Dump: ");
    for (int q = 0; q < 4; q++) {
        printf("%.02X ", dd[q]);
    }

    printf("\n");
    return;
}
// ----------------------------------------------------------
void broadcomGetHIDDevices(int ic, unsigned char address, bool del = false) {
    if (address);

    printf("Broadcom GET Report\n");

    UHCI_QH far * qh;
    UHCI_TD far * td;
    UHCI_TD far * td1;
    UHCI_TD far * td2;
    UHCI_TD far * td3;
    UHCI_TD far * td4;
    UHCI_TD far * stop;

    unsigned char far * dd;

    dd = (unsigned char far*)align_malloc(64, 16);
    _fmemset(dd, 0, 64);

    qh = (UHCI_QH far *)align_malloc(32, 16);
    td = (UHCI_TD far *)align_malloc(32, 16);
    td1 = (UHCI_TD far *)align_malloc(32, 16);
    td2 = (UHCI_TD far *)align_malloc(32, 16);
    td3 = (UHCI_TD far *)align_malloc(32, 16);
    td4 = (UHCI_TD far *)align_malloc(32, 16);

    stop = (UHCI_TD far *)align_malloc(32, 16);

    uhciSetupPacket far * info = (uhciSetupPacket far *)align_malloc(32, 8);
    info->requestType = 0xA1; // D-H I C DEVICE_TO_HOST | SETUP_INTERFACE | INTERFACE_QUERY;
    info->requestID = 0x01; // custom query (SETUP)
    info->wValue = 0x37E;
    info->wIndex = 0;
    info->wLength = 28;

    td->link_ptr = (_faddr(td1) & 0xFFFFFFF0) | 0x04;
    td->stats = (TRIPLE_ERROR | ACTIVE);
    td->info = ((7L << 21) | DATA0 | SETUP | ((address & 0x7F) << 8));
    td->buffer = _faddr(info);

    td1->link_ptr = (_faddr(td2) & 0xFFFFFFF0) | 0x04;
    td1->stats = (TRIPLE_ERROR | ACTIVE);
    td1->info = ((7L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td1->buffer = _faddr(dd);

    td2->link_ptr = (_faddr(td3) & 0xFFFFFFF0) | 0x04;
    td2->stats = (TRIPLE_ERROR | ACTIVE);
    td2->info = ((7L << 21) | DATA0 | IN | ((address & 0x7F) << 8));
    td2->buffer = _faddr(dd + 8);

    td3->link_ptr = (_faddr(td4) & 0xFFFFFFF0) | 0x04;
    td3->stats = (TRIPLE_ERROR | ACTIVE);
    td3->info = ((7L << 21) | DATA1 | IN | ((address & 0x7F) << 8));
    td3->buffer = _faddr(dd + 16);

    td4->link_ptr = (_faddr(stop) & 0xFFFFFFF0) | 0x04;
    td4->stats = (TRIPLE_ERROR | ACTIVE);
    td4->info = ((3L << 21) | DATA0 | IN | ((address & 0x7F) << 8));
    td4->buffer = _faddr(dd + 24);

    stop->link_ptr = (1L << 2) | 1;
    stop->stats = (TRIPLE_ERROR | ACTIVE);
    stop->info = ((0x7FFL << 21) | DATA1 | OUT | ((address & 0x7F) << 8));
    stop->buffer = 0;

    qh->horz = 1;
    qh->vert = _faddr(td);

    unsigned long currentStats = stop->stats;

    printf("Status: %.04lX\n", currentStats);

    uhciSetFrameListItem(UHCIControllers[ic].pFrameList, 0, qh, 0x02);
    uhciResetFrameNumber(ic);

    int count = 0;
    do {
        if (!(td->stats & ACTIVE))
            printf("TD:done\r");

        if (!(td1->stats & ACTIVE))
            printf("TD1:done\r");

        if (!(td2->stats & ACTIVE))
            printf("TD2:done\r");

        if (!(td3->stats & ACTIVE))
            printf("TD3:done\r");

        if (!(td4->stats & ACTIVE))
            printf("TD4:done\r");

        if (currentStats != stop->stats) {
            currentStats = stop->stats;
            printf("Status:%.04lX\n", currentStats);
        }
        count++;
        delay(1);
    } while ((currentStats & (1L << 23)) && (count < UHCI_TIMEOUT));

    delay(100);

    if (count == UHCI_TIMEOUT) {
        printf("broadcomGetHIDDevices: Timeout\n");
        printf("Dump: ");
        for (int q = 0; q < 28; q++) {
            printf("%.02X ", dd[q]);
        }
        return;
    }
    else {
        printf("broadcomGetHIDDevices: Operation complete\n");
    }

    printf("Dump: ");
    for (int q = 0; q < 28; q++) {
        printf("%.02X ", dd[q]);
    }

    if (del) {
        broadcomDeleteDevice(ic, address, dd[12],
            dd[13], dd[14], dd[15], dd[16], dd[17]);

        broadcomDeleteDevice(ic, address, dd[20],
            dd[21], dd[22], dd[23], dd[24], dd[25]);

    }
    printf("\n");
    return;
}
// ----------------------------------------------------------

void uhciResetPort(unsigned int ic, unsigned int port) {
    outpw(UHCIControllers[ic].USBBaseAddr + port, 0x200); delay(100);
    outpw(UHCIControllers[ic].USBBaseAddr + port, 0x000);
    delay(100);
}
// ----------------------------------------------------------

int uhciFind() {
    unsigned char currentDevID = 0;
    unsigned char returnCode;
    unsigned char __BH;
    unsigned char __BL;
    unsigned char retCode;

    do {
        returnCode = 0;

        _CL = _UHCI;
        _SI = currentDevID;
        _ECX = 0x000c0300;
        _AX = 0x0B103;

        asm{
            stc
        }
        geninterrupt(0x1A);
        retCode = _AH;
        __BH = _BH;
        __BL = _BL;

        if (retCode)
            break;

        uhciAddController(__BH, __BL);
        currentDevID++; // check next device
    } while (!returnCode);
    return (nUHCIFound != 0);
};
//-----------------------------------------------------------------------
void uhciListControllers() {
    int i = 0;
    for (i = 0; i < nUHCIFound; i++) {
        printf(" ------- UHCI controller ID:%02X --------- \n");

        printf("USB Controller Device:%02X, Bus:%02X\n",
            UHCIControllers[i].USBBusNum,
            UHCIControllers[i].USBDevNum);
        printf("UHCI Controller VendorID:%04X\n",
            UHCIControllers[i].USBVendorID);
        printf("UHCI Controller DeviceID:%04X\n",
            UHCIControllers[i].USBDeviceID);
        printf("UHCI Controller IRQ:%02X\n",
            UHCIControllers[i].USBIrqNum);
        printf("UHCI Controller Base Address:%04X\n",
            UHCIControllers[i].USBBaseAddr);
        printf("---------------------------------------------------\n");
    }
}

//-------------------------------------------------------------------------
int isPCI(void) {
    int bRetValue = 0;

    asm{
        push ax
        push bx
        push cx
        mov ax, 0xb101
        xor dx,dx
        xor di,di
        stc
        int 0x1A
        jc  nopci
    }

    bRetValue = 1;

nopci:
    asm{
        pop cx
        pop bx
        pop ax
    }
    return bRetValue;
}

int is8086() {
    asm{
         push ax
         push bx
         pushf
         mov bx, 0xf000
         pushf
         pop ax
         and ax, 0xfff
         push ax
         popf
         pushf
         pop ax
         and ax, bx
         cmp ax, bx
         je cpu8086
         popf
         pop bx
         pop ax
    }
    return 0;
cpu8086:
    asm{
         pop bx
         pop ax
    }
    return 1;
}

int is80386(void) {
    asm{
        push ax
        push bx
        mov bx, 0xf000
        pushf
        pop ax
        or ax,bx
        push ax
        popf
        pushf
        pop ax
        test ax,bx
        jnz cpu80386
        popf
        pop bx
        pop ax
    }
    return 0;
cpu80386:
    asm{
        pop bx
        pop ax
    }
    return 1;
}


void waitForKey() {
    return;
    printf("Press any key to continue...\n");
    while (!kbhit());
    getch();
}

int main(int argc, char ** argv) {
    printf("USB Broadcom scanner\n");

    if (argc);
    if (argv);

    printf("8086 CPU  ........ %s", ((b8086 = is8086()) == 1) ? "yes\n" : "no\n");
    if (b8086) {
        printf("Need at least 80386+ to continue...\n");
        return -1;
    }
    log("8086 check is OK");

    printf("80386+ CPU ....... %s", ((b80386 = is80386()) == 1) ? "yes\n" : "no\n");
    if (!b80386) {
        printf("Need at least 80386+ to continue...\n");
        return -1;
    }
    log("80386 check is OK");

    bPCI = isPCI();
    printf("PCI BIOS check ..%s", bPCI ? "present\n" : "not present\n");
    if (!bPCI) {
        printf("PCI BIOS not present\n");
        return -1;
    }

    uhciFind();
    if (!nUHCIFound) {
        printf("No compatible USB controllers found\n");
        return -1;
    }

    uhciListControllers();
    waitForKey();

    unsigned char deviceNumber = 10;
    unsigned char bcDev;

    for (int i = 0; i < nUHCIFound; i++) {
        uhciTakeFromBIOS(i);
        uhciReset(i);
        // allocate memory for frame list
        uhciInitFramesList(i);

        // send schedule plan to controller
        uhciSetFrameListAddress(i);

        unsigned long addr = uhciGetFrameListAddress(i);
        printf("Address value in Register:%.04lX\n", addr);
        uhciRun(i);

        uhciResetPort(i, PORTSC0);
        uhciEnablePort(i, PORTSC0);

        unsigned int portStatus = uhciIsDeviceConnected(i, PORTSC0);

        if (portStatus == LOW_SPEED_CONNECTED) {
            printf("Low speed device present on PORT0\n");
        }
        else if (portStatus == HIGH_SPEED_CONNECTED) {
            printf("High speed device present on PORT0\n");
        }
        else {
            printf("PORT0 is Empty\n");
        }
        // process port
        if (portStatus != PORT_EMPTY) {

            uhciDescriptor descr;
            if (uhciGetDescriptor(i, 0, &descr))
                uhciDumpDescriptor(&descr);

            waitForKey();

            uhciSetAddress(i, deviceNumber);

            waitForKey();

            if (uhciGetDescriptor(i, deviceNumber, &descr)) {
                
                uhciDumpDescriptor(&descr);
                if ((descr.idVendor == 0x0A5C) &&
                    (descr.idProduct == 0x4500)) {
                    printf("Broadcom HUB detected\n");

                    printf("Setting up configuration\n");
                    uhciSetConfiguration(i, deviceNumber, 1);

                    int p = 0;
                    bcDev = deviceNumber + 1;
                    for (p = 1; p < 4; p++) {
                        uhciHubStatus ps;
                        uhciHubClearFeature(i, deviceNumber, p);
                       
                        printf("Enabling power on port %d\n", p);
                        uhciHubPortPowerOn(i, deviceNumber, p);
                        uhciHubResetPort(i, deviceNumber, p);
                       
                        delay(200);
                        uhciGetHubStatus(i, deviceNumber, p, &ps);
                        if (!(ps.statusBits & 0x01)) {
                            printf("Port is empty, skipping\n");
                            continue;
                        }
                       
                        if (uhciGetDescriptor(i, 0, &descr)) {
                            uhciDumpDescriptor(&descr);
                            bcDev++;
                            uhciSetAddress(i, bcDev);
                       
                            if (descr.idVendor == 0x0A5C &&
                                descr.idProduct == 0x4503) {
                                printf("First broadcom device found (%d)\n", bcDev);

                                uhciSetConfiguration(i, bcDev, 1);

                                broadcomSetReport(i, bcDev);
                                waitForKey();
                                broadcomGetHIDDevices(i, bcDev, true);
                                waitForKey();                     
                            }

                            if (descr.idVendor == 0x0A5C &&
                                descr.idProduct == 0x4502) {
                                printf("Second broadcom device found (%d)\n", bcDev);
                            }
                        }

                        waitForKey();
                    }
                }
            }
            deviceNumber++;

        }

        waitForKey();
        uhciResetPort(i, PORTSC1);
        uhciEnablePort(i, PORTSC1);

        portStatus = uhciIsDeviceConnected(i, PORTSC1);
        if (portStatus == LOW_SPEED_CONNECTED) {
            printf("Low speed device present on PORT1\n");
        }
        else if (portStatus == HIGH_SPEED_CONNECTED) {
            printf("High speed device present on PORT1\n");
        }
        else {
            printf("PORT1 is Empty\n");
        }

        // process port
        if (portStatus != PORT_EMPTY) {
            uhciDescriptor descr;

            if (uhciGetDescriptor(i, 0, &descr))
                uhciDumpDescriptor(&descr);

            waitForKey();

            printf("Going to set address to: (%d)\n", deviceNumber);

            uhciSetAddress(i, deviceNumber);

            waitForKey();

            if (uhciGetDescriptor(i, deviceNumber, &descr)) {
                uhciDumpDescriptor(&descr);
                if ((descr.idVendor == 0x0A5C) &&
                    (descr.idProduct == 0x4500)) {
                    printf("Broadcom HUB detected\n");

                    printf("Setting up configuration\n");
                    uhciSetConfiguration(i, deviceNumber, 1);

                    int p;
                    bcDev = deviceNumber + 1;
                    for (p = 1; p < 4; p++) {
                        uhciHubStatus ps;
                        uhciHubClearFeature(i, deviceNumber, p);
                       
                        printf("Enabling power on port %d\n", p);
                        uhciHubPortPowerOn(i, deviceNumber, p);
                        uhciHubResetPort(i, deviceNumber, p);
                       
                        delay(200);
                        uhciGetHubStatus(i, deviceNumber, p, &ps);
                        if (!(ps.statusBits & 0x01)) {
                            printf("Port is empty, skipping\n");
                            continue;
                        }
                        
                        if (uhciGetDescriptor(i, 0, &descr)) {
                            uhciDumpDescriptor(&descr);
                            bcDev++;
                            uhciSetAddress(i, bcDev);
                        
                            if (descr.idVendor == 0x0A5C &&
                                descr.idProduct == 0x4503) {
                                printf("First broadcom device found (%d)\n", bcDev);

                                uhciSetConfiguration(i, bcDev, 1);

                                broadcomSetReport(i, bcDev);
                                waitForKey();
                                broadcomGetHIDDevices(i, bcDev, true);
                                waitForKey();                        
                            }

                            if (descr.idVendor == 0x0A5C &&
                                descr.idProduct == 0x4502) {
                                printf("Second broadcom device found (%d)\n", bcDev);
                            }
                        }

                        waitForKey();
                    }
                }

            }
            deviceNumber++;


        }
    }

    printf("OK\n");
    return 0;
}