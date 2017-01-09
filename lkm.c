#include <linux/kernel.h>   //Для printk()
#include <linux/module.h>   //Модуль __init, __exit ....
#include <linux/init.h>     //Определения макросов
#include <linux/fs.h>
#include <asm/uaccess.h>    //Для put_user()
#include <linux/kthread.h>  //thread
#include <linux/delay.h>    //msleep() 
#include <linux/slab.h>     //kmalloc()
#include <linux/sched.h>    //task_struct, put_task_struct
#include <linux/time.h>		//current time

//Информация о модуле, которую можно будет увидеть с помощью Modinfo
MODULE_LICENSE("LGPL");
MODULE_AUTHOR("Developer by <Dovgon@yandex.ru>");
MODULE_DESCRIPTION("System info module");
MODULE_SUPPORTED_DEVICE("sysinfo");             // /dev/sysinfo

#define DEVICE_NAME "sysinfo"   //Имя устройства
#define SIZE_BUFFER_DEVICE 65536 //размер буфера

//Прототифы функций поддерживаемые нашим устройством (открытие, закрытие, чтение, запись)
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);

//Прототипы функций работы с файлом (открытие, чтение)
struct file *file_open(const char *path, int flags, int rights);
int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size);

//Прототипы функций, вспомогательных операций (построчное чтнеие из файла, запись строки в конец буфера)
int  fgets(unsigned char *name, struct file *fp, int iRead_offset);//Чтение из файла, по строчна
void putBufferDevice(char *message, int size);       //Запись в буфер новых данных

//Глобальные переменные
static int superior_number;         //Старший номер устройства, нашего устройсва
static int isDeviceOpen=0;          //Используется устройства 0 - нет
static char *textUser;              //Данные отоброжаемые пользователю
static char *p_textUser;            //указатель на начало буфера
static int pointText=0;             //Последний символ в буфере
static int bRang=1;                 //переменная количества расчетов загрузки процессора

struct task_struct *taskThread;

//структура операций над устройством
static struct file_operations fops={
  .read=device_read,
  .write=device_write,
  .open=device_open,
  .release=device_release
};

///////////////////////////////////////////////////////////////////////////////////////////

//загрузка процессора и каждого из его ядра в отдельности 0 - общая по процессора 1 - 1-ое ядро ....
typedef struct{
    unsigned long usage[16];
} proc_info_t;

//сбор информации по загрузки процессора, итерация
long rangCPUInfo(proc_info_t *info){
    struct file *fp;
    unsigned char tempRead[1024];
    int iRead_offset=0, iStrLen, index=0, iCPU;
    
    static unsigned long pre_total[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, pre_used[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    unsigned long cpu[16], nice[16], system[16], idle[16];
    unsigned long used[16], total[16];
    
    //открываем файл с данными по загрузки процессора
    fp=file_open("/proc/stat", O_RDONLY, 0);
    
    //читаем по строчно
    do{
        //читаес строку из файла
        iStrLen=fgets(tempRead, fp, iRead_offset);
        if(iStrLen==0)  //если достигнут конец файла
            break;
        iRead_offset+=iStrLen;  //увеличиваем смещение от начала файла
        
        //если первые символы в строке 'cpu'
        if(tempRead[0]=='c' && tempRead[1]=='p' && tempRead[2]=='u'){
            sscanf(tempRead, "%*s %lu %lu %lu %lu", &cpu[index], &nice[index], &system[index], &idle[index]);
            index++;
        }
    }while(true);
    
    //закрываем файл
    filp_close(fp, NULL);
    
    //расчтет по процессору и каждому ядру в отдельности
    for(iCPU=0; iCPU<index; iCPU++){
        used[iCPU]=cpu[iCPU]+nice[iCPU]+system[iCPU];
        total[iCPU]=cpu[iCPU]+nice[iCPU]+system[iCPU]+idle[iCPU];

        if(pre_total[iCPU]==0 || pre_used[iCPU]==0) //если первый расчет
            info->usage[iCPU]=0;
        else                                        //если второй и последующие
            info->usage[iCPU]=100*(used[iCPU]-pre_used[iCPU])/(total[iCPU]-pre_total[iCPU]);

        //данные текущего расчтета будут использоватся для следуйщего
        pre_used[iCPU]=used[iCPU];
        pre_total[iCPU]=total[iCPU];
    }
    
    return index;
}

//информация по загрузки процессора
void infoCPU(void){
    proc_info_t info;
    char sBuffer[256];
    int index, lenString, iCPU;
    struct timespec curr_tm;
    
    //выполняем чтение и расчет
    index=rangCPUInfo(&info);
    
    if(bRang==2){   //если есть данные предведущего расчета
        getnstimeofday(&curr_tm);
        for(iCPU=0; iCPU<index; iCPU++){
            lenString=sprintf(sBuffer, "TIME %.2lu:%.2lu:%.2lu:%.6lu - Load CPU%d %lu%%\n", 
                                (curr_tm.tv_sec/3600)%24, (curr_tm.tv_sec/60)%60, curr_tm.tv_sec%60, curr_tm.tv_nsec/1000,
                                iCPU, info.usage[iCPU]);
            putBufferDevice(sBuffer, lenString);
        }
    }
    else    //если еще был обин расчет
        bRang=2;
}

void infoRAM(void){
    char tempRead[256], sBuffer[256];
    int lenString, iRead_offset=0, iStrLen;
    struct file *fp;
    struct timespec curr_tm;

    //получаем текущее значения времени
    getnstimeofday(&curr_tm);

    //открываем файл инф о состоянии памяти
    fp=file_open("/proc/meminfo", O_RDONLY, 0);
    
    do{
        //читаем по строчно
        iStrLen=fgets(tempRead, fp, iRead_offset);
        if(iStrLen==0)  //если достигнут конец файла
            break;
        iRead_offset+=iStrLen;  //увеличиваем смещение в файле

        //если строка начинаетсяс 'MemTotal'
        if(tempRead[0]=='M' && tempRead[1]=='e' && tempRead[2]=='m' && tempRead[3]=='T' && tempRead[4]=='o' && tempRead[5]=='t' && 
            tempRead[6]=='a' && tempRead[7]=='l'){
                lenString=sprintf(sBuffer, "TIME %.2lu:%.2lu:%.2lu:%.6lu - %s\n", 
                                    (curr_tm.tv_sec/3600)%24, (curr_tm.tv_sec/60)%60, curr_tm.tv_sec%60, curr_tm.tv_nsec/1000,
                                    tempRead);
                putBufferDevice(sBuffer, lenString);    //сохраняем в буфер
        }
        //если строка начинаетсяс 'MemFree'
        if(tempRead[0]=='M' && tempRead[1]=='e' && tempRead[2]=='m' && tempRead[3]=='F' && tempRead[4]=='r' && tempRead[5]=='e' && 
            tempRead[6]=='e'){
                lenString=sprintf(sBuffer, "TIME %.2lu:%.2lu:%.2lu:%.6lu - %s\n", 
                                    (curr_tm.tv_sec/3600)%24, (curr_tm.tv_sec/60)%60, curr_tm.tv_sec%60, curr_tm.tv_nsec/1000,
                                    tempRead);
                putBufferDevice(sBuffer, lenString);    //сохраняем в буфер
        }
    }while(true);
    
    //закрываем файл
    filp_close(fp, NULL);
}

void infoPS(void){
    struct task_struct *g, *p;
    char sBuffer[256];
    int iBuf;
    struct timespec curr_tm;

    //получаем текущее значения времени
    getnstimeofday(&curr_tm);
    
    //получаем структура с инф по процессу
    do_each_thread(g, p){
        iBuf=sprintf(sBuffer, "TIME  %.2lu:%.2lu:%.2lu:%.6lu -Task %s (pid = %d)\n", 
                        (curr_tm.tv_sec/3600)%24, (curr_tm.tv_sec/60)%60, curr_tm.tv_sec%60, curr_tm.tv_nsec/1000,
                        p->comm, task_pid_nr(p));
        putBufferDevice(sBuffer, iBuf); //сохраняем в буфер
    }while_each_thread(g, p);
}

int fgets(unsigned char *name, struct file *fp, int iRead_offset){
    int readByte=0;
    char tempChar;

    while(true){
        if(file_read(fp, iRead_offset, &tempChar, 1)!=0 && tempChar!='\n')      //если не конец файла и прочитанный символ не переход на новую строку, сохр. в буфер
            name[readByte]=tempChar;
        else                                                                    //иначе выходим из цикла
            break;
        ++iRead_offset;
        ++readByte;
    }
    
    if(readByte==0) //если нет прочитанных символов 
        return 0;

    //последний символ 0
    ++readByte;
    name[readByte]=0;

    return readByte;
}

void putBufferDevice(char *message, int lenString){
    int iBuf=0;
    
    //
    for(; iBuf<lenString; iBuf++){
        if(pointText==SIZE_BUFFER_DEVICE)   //если достигнут конец буфера, начинаем писать с начало
            pointText=0;                    //устанавливаем указатель в начало буфера

        textUser[pointText]=message[iBuf];  
        ++pointText;
    }
    
    if(pointText==SIZE_BUFFER_DEVICE)   //если достигнут конец буфера, начинаем писать с начало
        textUser[0]=0;                  //обнуляем буфер
    else                                //устанавливаем последний символ 0 в буфере
        textUser[pointText]=0;
}

struct file *file_open(const char *path, int flags, int rights){
    struct file *fp=NULL;
    mm_segment_t oldfs;
    int err=0;
    
    //выделяем область в памяти для отоброжения файла
    oldfs=get_fs();
    set_fs(get_ds());
    //открываем файл
    fp=filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(fp)){ //если ошибка открытия
        printk("*** sysinfo - error open file - %s\n", path);
        err=PTR_ERR(fp);
        return NULL;
    }

    return fp;
}

int file_read(struct file *file, unsigned long long offset, unsigned char *data, unsigned int size){
    mm_segment_t oldfs;
    int ret;
    
    //файл не открыт
    if(file==NULL)
        return 0;
    
    oldfs=get_fs();
    set_fs(get_ds());

    //чтения данных из файла
    ret=vfs_read(file, data, size, &offset);
    set_fs(oldfs);

    return ret;
}

static int thread_sysinfo(void *data){
    while(!kthread_should_stop()){
        //cpu
        infoCPU();
        
        //ram
        infoRAM();
        
        //proccess
        infoPS();

        //задержка перед след. расчетом
        msleep(1000);
    }

    return 0;
}

// Функция загрузки модуля
static int __init sysinfo_init(void){
    printk(KERN_ALERT "*** sysinfo module loaded...");

    //Регистрируем устройсво и получаем старший номер устройства
    superior_number=register_chrdev(0, DEVICE_NAME, &fops);
    if(superior_number<0){
        printk("FAILED\n*** sysinfo - Registering the character device failed with %d\n", superior_number);
        return superior_number;
    }
    
    //Выделяем буффер
    textUser=kmalloc(SIZE_BUFFER_DEVICE, GFP_KERNEL);

    printk("OK\n");
    //Сообщаем присвоенный старший номер устройства
    printk("*** sysinfo - Please, create a dev file with 'mknod /dev/sysinfo c %d 0'.\n", superior_number);
    
    //Запускаем поток, для отслеживания состояния системы
    taskThread=kthread_run(thread_sysinfo, NULL, "thread_sysinfo");

    return 0;
}

// Функция выгрузки модуля
static void __exit sysinfo_exit(void){
    printk(KERN_ALERT "sysinfo module is unloaded...");
    
    //Уничтожение буфера
    printk(" *** sysinfo - Buffer free\n");
    kfree(textUser);
    
    //Остонавливаем поток
    printk("*** sysinfo - Thread stop\n");
    kthread_stop(taskThread);
    
    //Освобождаем устройство
    printk("*** sysinfo - destroy device\n");
    unregister_chrdev(superior_number, DEVICE_NAME);
}

//Указываем функции загрузки и выгрузки
module_init(sysinfo_init);
module_exit(sysinfo_exit);

static int device_open(struct inode *inode, struct file *file){    
    if(isDeviceOpen)
        return -EBUSY;
    isDeviceOpen++;

    //указатель на буфер в начало
    p_textUser=textUser;

    return 0;
}

static int device_release(struct inode *inode, struct file *file){
    isDeviceOpen--;
    return 0;
}

static ssize_t device_write(struct file *filp, const char *buff, size_t len, loff_t * off ){
    printk("Sorry, this operation isn't supported.\n");
    return -EINVAL;
}

static ssize_t device_read(struct file *filp, char *buffer, size_t length, loff_t *offset){
    int byte_read=0;

    //если текса нет
    if(pointText==0)
        return 0;

    //выводим пока есть буфер и не закончиля текст
    while(length && *p_textUser){
        //выводим
        put_user(*(p_textUser++), buffer++);
        
        length--;
        byte_read++;
    }
    
    return byte_read;
}
