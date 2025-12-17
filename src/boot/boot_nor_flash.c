typedef struct tagFlashFile{
	unsigned int MagicCode;
	unsigned int fileID;
	void * baseAddr;
	unsigned int fileLen;
	unsigned int CRC;
	unsigned int ptrOffset;
	char fileName[128];
}FLASHFILE;

void Do_nor_flash_init(void) 
{
    FlashInit();
}

int NOR_CheckForNewFirmware(void) 
{
    return 1;
}

void NOR_ReadFirmwareAndWriteToInternalFlash(unsigned int cpu_address)
{
    FLASHFILE cpu0;
    QspiFlashRead(0x30000, &cpu0, sizeof(FLASHFILE));
}

/*
void copy_nor_flash_to_internal_flash(void)
{
    uint32_t nor_flash_src_addr = 0x60000000; // NOR Flash Address
    uint32_t internal_flash_dst_addr = 0x08008000; // cpu flash address
    
    
    uint32_t data_size = 1024; //file size
    uint8_t buffer[1024];

    // 1. ? NOR Flash ????? RAM
    memcpy(buffer, (void*)nor_flash_src_addr, data_size);

    // 2. ???? Flash ??(STM32F7 Sector 2)
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_2;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &sector_error);

    // 3. ??????? Flash
    for (uint32_t i = 0; i < data_size; i += 4) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, internal_flash_dst_addr + i, *(uint32_t*)(buffer + i));
    }
    HAL_FLASH_Lock();

    // 4. ????
    uint32_t *flash_data = (uint32_t*)internal_flash_dst_addr;
    for (uint32_t i = 0; i < data_size/4; i++) {
        if (flash_data[i] != ((uint32_t*)buffer)[i]) {
            // ????
            break;
        }
    }
}*/