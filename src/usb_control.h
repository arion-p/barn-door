#ifndef USB_CONTROL_H
#define USB_CONTROL_H

#define COMMAND_BUFFER_SIZE    64
#define LINE_TERMINATOR '\r'


class UsbControl {
    public:
        UsbControl() {
            bufferLen = 0;
            buffer[0] = 0;
        }

        void loop(long timeleft);

    protected:
        char buffer[COMMAND_BUFFER_SIZE];
        int bufferLen;

        void processLine();

};

#endif
