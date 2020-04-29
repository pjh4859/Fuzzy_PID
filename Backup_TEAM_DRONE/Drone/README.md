## DRONE FLIGHT PART
We use NUCLEOf411RE board to Flight Controller.

### UART DMA comment
이렇게 코드 짜시면... 안됩니다... 버퍼 내용 다 보냈는지 while로 확인해서 기다릴 거면... DMA를 쓰는 이유가 없지요. \
DMA 버퍼 전송 시작 전에 Busy 플래그 같은거 만들어서 셋업 시킨 다음 DMA 전송 완료 인터럽트 또는\
Empty 인터럽트 인에이블 시켜서 전송 완료 되면 Busy 플래그 클리어 시키고 또한 전송 루틴에서는 \
플래그 확인해서 클리어 되어 있으면 전송 하도록 루틴을 구성해야 합니다. While 문 같은거로 기다리고 하는 것 없이요. \
그리고 속도 때문에 지정 시간내에 다 보낼 수 없으면 Ring Buffer 구현해서 보내면 됩니다.
