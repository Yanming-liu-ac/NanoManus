#include <cstdlib>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cctype>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <stack>
#include <windows.h>
#include <process.h>
#include <string.h>
#include <string>
#include <math.h>
#include <process.h>
#include <ldata.h>  // 必须包含L-Edit UPI头文件

// Simple C Network Receiver using Custom DLL - UTF-8 Encoded
// Pure thread architecture: Main thread handles business logic, Socket thread handles network


// Simple C Network Receiver using Custom DLL - UTF-8 Encoded
// Multi-process architecture: Main process handles business logic, Child process handles socket


// DLL function pointer type definitions
typedef int(__stdcall* InitNetworkFunc)(void);
typedef void* (__stdcall* CreateSocketFunc)(void);  // Use void* to avoid type conflicts
typedef int(__stdcall* BindSocketFunc)(void* sock, void* addr);
typedef int(__stdcall* ListenSocketFunc)(void* sock, int backlog);
typedef void* (__stdcall* AcceptConnectionFunc)(void* sock, void* addr, int* addrlen);
typedef int(__stdcall* ReceiveDataFunc)(void* sock, char* buffer, int buflen);
typedef int(__stdcall* SendDataFunc)(void* sock, const char* buffer, int buflen);
typedef int(__stdcall* CloseSocketFunc)(void* sock);
typedef void(__stdcall* CleanupNetworkFunc)(void);
typedef unsigned short(__stdcall* HtonsFunc)(unsigned short hostshort);

// Simple address structure
struct simple_addr {
    short family;           // Address family
    unsigned short port;    // Port number
    unsigned long ip;       // IP address
    char zero[8];          // Padding
};

// Global DLL function pointers
InitNetworkFunc init_network;
CreateSocketFunc create_socket;
BindSocketFunc bind_socket;
ListenSocketFunc listen_socket;
AcceptConnectionFunc accept_connection;
ReceiveDataFunc receive_data;
SendDataFunc send_data;
CloseSocketFunc close_socket;
CleanupNetworkFunc cleanup_network;
HtonsFunc dll_htons;

// 功能队列系统结构定义
struct MacroFunction {
    char name[64];                  // 功能名称
    void (*function_ptr)(void);     // 函数指针
    char description[256];          // 功能描述
    int enabled;                   // 是否启用
};

// 版图信息结构
struct LayoutInfo {
    float width;                   // 宽度
    float length;                  // 长度
    float x_position;              // X坐标
    float y_position;              // Y坐标
    char layer_info[512];          // 层信息
    int polygon_count;             // 多边形数量
    char cell_name[128];           // 单元名称
    char timestamp[64];            // 时间戳
};

// 功能执行结果结构
struct MacroResult {
    int success;                   // 执行是否成功
    char error_message[256];       // 错误信息
    LayoutInfo layout_data;        // 版图信息
    char custom_response[512];     // 自定义响应内容
};

// 全局功能队列
#define MAX_MACRO_FUNCTIONS 20
MacroFunction g_macroQueue[MAX_MACRO_FUNCTIONS];
int g_macroQueueSize = 0;

// 全局结果存储
MacroResult g_lastResult = {0};

// Thread-based communication structure
struct ThreadMessage {
    int type;        // 1: data from client, 2: response to client, 3: shutdown
    int length;      // message length
    char data[2048]; // message content - 增加到2048字节以支持更长的响应
    DWORD timestamp; // message timestamp
};

// Message queue structure
#define MAX_QUEUE_SIZE 100
struct MessageQueue {
    ThreadMessage messages[MAX_QUEUE_SIZE];
    int head;
    int tail;
    int count;
    CRITICAL_SECTION cs;
    HANDLE hNotEmpty;  // Event for queue not empty
    HANDLE hNotFull;   // Event for queue not full
};

// Thread communication handles
HANDLE g_hSocketThread = NULL;
HANDLE g_hMessageThread = NULL;  // Message processing thread
MessageQueue g_MainToSocket = {0};  // Main thread to Socket thread - zero initialize
MessageQueue g_SocketToMain = {0};  // Socket thread to Main thread - zero initialize
volatile int g_threadShutdown = 0;

// Main thread timer for checking pending transistor execution
UINT_PTR g_mainThreadTimerID = 0;

// Function declarations - C++ style (with C++ types)
int init_message_queue(MessageQueue* queue);
void cleanup_message_queue(MessageQueue* queue);
int send_message(MessageQueue* queue, const ThreadMessage* msg);
int receive_message(MessageQueue* queue, ThreadMessage* msg, DWORD timeout);
unsigned __stdcall socket_thread_function(void* arg);
unsigned __stdcall message_processing_thread_function(void* arg);
int start_socket_thread(void);
int start_message_thread(void);
void cleanup_threads(void);
int check_and_process_socket_messages(void);
int socket_thread_main(void);
int main_thread_logic(void);
int simple_start_server(void);

// 功能队列管理函数声明
int register_macro_function(const char* name, void (*function_ptr)(void), const char* description);
int execute_macro_by_name(const char* name);
void list_available_macros(char* result_buffer, int buffer_size);
void extract_layout_info(LayoutInfo* info);
void format_macro_result(const MacroResult* result, char* response_buffer, int buffer_size);
void enhanced_transistor_macro(const char* params);
void memristor_macro(const char* params);
void parse_command_with_params(const char* command, char* cmd_name, char* params);
void initialize_macro_queue_system(void);

// Timer callback function for main thread execution
void CALLBACK MainThreadTimerProc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime);

// C-style declarations for UPI
extern "C" {
    void MyMacro(void);
    void TransistorMacro(void);  // Add declaration for TransistorMacro function
    void check_and_execute_transistor_macro(void);  // New function for main thread execution
    void PolygonMacro(const char* params); // Add declaration for PolygonMacro function
    void PathMacro(const char* params); // Add declaration for PathMacro function
    void TextMacro(const char* params); // Add declaration for TextMacro function
    void CircleMacro(const char* params); // Add declaration for CircleMacro function
    void PieWedgeMacro(const char* params); // Add declaration for PieWedgeMacro function
    void RingWedgeMacro(const char* params); // Add declaration for RingWedgeMacro function
    void BoolExprMacro(const char* params); // Add declaration for BoolExprMacro function
    void ArrayMacro(const char* params); // Add declaration for ArrayMacro function
    int UPI_Entry_Point(void);
    void log_main(const char* message);
    void log_socket(const char* message);
    int start_network_server(void);
    void socket_main(void);
    void stop_socket_server(void);  // New function to stop server
    int is_shutdown_requested(void);
    void start_main_thread_timer(void);  // Add timer functions to UPI
    void stop_main_thread_timer(void);
}


// Main entry function - pure thread-based architecture - Enhanced with macro queue system
int start_network_server(void) {
    log_main("=== Network Server Starting (Pure Thread Architecture with Macro Queue System) ===");
    
    // Check if server is already running
    if (g_hSocketThread != NULL || g_hMessageThread != NULL) {
        log_main("WARNING: Server is already running, stopping previous instance first");
        cleanup_threads();
        Sleep(200);  // Give time for cleanup
    }
    
    log_main("Using thread-based architecture with message queues and macro queue system");
    
    // Initialize macro queue system first
    initialize_macro_queue_system();
    
    // Initialize message queues
    if (init_message_queue(&g_MainToSocket) != 0) {
        log_main("ERROR: Failed to initialize main-to-socket message queue");
        return -1;
    }
    if (init_message_queue(&g_SocketToMain) != 0) {
        log_main("ERROR: Failed to initialize socket-to-main message queue");
        cleanup_message_queue(&g_MainToSocket);
        return -1;
    }
    log_main("Message queues initialized successfully");
    
    // Start socket thread
    if (start_socket_thread() != 0) {
        log_main("ERROR: Failed to start socket thread");
        cleanup_message_queue(&g_MainToSocket);
        cleanup_message_queue(&g_SocketToMain);
        return -1;
    }
    log_main("Socket thread started successfully");
    
    // Start message processing thread
    if (start_message_thread() != 0) {
        log_main("ERROR: Failed to start message processing thread");
        cleanup_threads();
        return -1;
    }
    log_main("Message processing thread started successfully");
    
    // Main thread logic now just does initial setup and returns
    int result = main_thread_logic();
    
    // DON'T CLEANUP! Let threads run in background
    log_main("Network server startup completed - threads running in background");
    log_main("Threads will continue running until program exit or manual stop");
    log_main("Enhanced functionality: Macro queue system available");
    log_main("Send 'list' command to see available functions");
    return result;
}

// Simplified entry point - always use thread architecture
int simple_start_server(void) {
    log_main("simple_start_server called - using pure thread architecture");
    return start_network_server();
}

// Standard main function with no parameters
// Message queue implementation
int init_message_queue(MessageQueue* queue) {
    if (!queue) return -1;
    
    // Clean up existing resources if any
    if (queue->hNotEmpty || queue->hNotFull) {
        cleanup_message_queue(queue);
    }
    
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    
    InitializeCriticalSection(&queue->cs);
    queue->hNotEmpty = CreateEventA(NULL, FALSE, FALSE, NULL);
    queue->hNotFull = CreateEventA(NULL, FALSE, TRUE, NULL);
    
    if (!queue->hNotEmpty || !queue->hNotFull) {
        DeleteCriticalSection(&queue->cs);
        if (queue->hNotEmpty) CloseHandle(queue->hNotEmpty);
        if (queue->hNotFull) CloseHandle(queue->hNotFull);
        return -1;
    }
    
    return 0;
}

void cleanup_message_queue(MessageQueue* queue) {
    if (!queue) return;
    
    // Signal shutdown to any waiting threads
    if (queue->hNotEmpty) SetEvent(queue->hNotEmpty);
    if (queue->hNotFull) SetEvent(queue->hNotFull);
    
    // Give a moment for threads to wake up
    Sleep(100);
    
    // Safely cleanup Critical Section
    if (queue->cs.DebugInfo != NULL) {  // Check if CS is initialized
        EnterCriticalSection(&queue->cs);
        queue->count = 0;
        queue->head = 0;
        queue->tail = 0;
        LeaveCriticalSection(&queue->cs);
        DeleteCriticalSection(&queue->cs);
        memset(&queue->cs, 0, sizeof(queue->cs));  // Clear CS memory
    }
    
    if (queue->hNotEmpty) {
        CloseHandle(queue->hNotEmpty);
        queue->hNotEmpty = NULL;
    }
    if (queue->hNotFull) {
        CloseHandle(queue->hNotFull);
        queue->hNotFull = NULL;
    }
}

int send_message(MessageQueue* queue, const ThreadMessage* msg) {
    if (!queue || !msg) return -1;
    
    // Use timeout instead of INFINITE to prevent deadlock
    DWORD result = WaitForSingleObject(queue->hNotFull, 1000);  // 1 second timeout
    if (result != WAIT_OBJECT_0) {
        log_socket("WARNING: send_message timeout or failed");
        return -1;  // Timeout or error
    }
    
    EnterCriticalSection(&queue->cs);
    
    if (queue->count >= MAX_QUEUE_SIZE) {
        LeaveCriticalSection(&queue->cs);
        log_socket("ERROR: Message queue full");
        return -1;
    }
    
    // Add message to queue
    queue->messages[queue->tail] = *msg;
    queue->messages[queue->tail].timestamp = GetTickCount();
    queue->tail = (queue->tail + 1) % MAX_QUEUE_SIZE;
    queue->count++;
    
    // Signal queue not empty
    SetEvent(queue->hNotEmpty);
    
    // If queue is full, don't signal not full
    if (queue->count < MAX_QUEUE_SIZE) {
        SetEvent(queue->hNotFull);
    }
    
    LeaveCriticalSection(&queue->cs);
    return 0;
}

int receive_message(MessageQueue* queue, ThreadMessage* msg, DWORD timeout) {
    if (!queue || !msg) return -1;
    
    // Wait for queue not empty
    DWORD result = WaitForSingleObject(queue->hNotEmpty, timeout);
    if (result != WAIT_OBJECT_0) {
        return -1; // Timeout or error
    }
    
    EnterCriticalSection(&queue->cs);
    
    if (queue->count == 0) {
        LeaveCriticalSection(&queue->cs);
        return -1;
    }
    
    // Get message from queue
    *msg = queue->messages[queue->head];
    queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
    queue->count--;
    
    // Signal queue not full
    SetEvent(queue->hNotFull);
    
    // If queue still has messages, signal not empty
    if (queue->count > 0) {
        SetEvent(queue->hNotEmpty);
    }
    
    LeaveCriticalSection(&queue->cs);
    return 0;
}

void socket_main(void) {
    log_main("socket_main function called - NON-BLOCKING network server startup");
    
    // Start the network server and return immediately
    int result = start_network_server();
    
    if (result == 0) {
        log_main("Network server startup completed successfully");
        log_main("Socket thread is running in background");
        log_main("GUI should remain responsive");
        log_main("Connect to localhost:8888 to test network functionality");
        
        // Automatically start the main thread timer for transistor execution
        start_main_thread_timer();
        log_main("Main thread timer started for automatic transistor execution");
    } else {
        char errorMsg[100];
        sprintf(errorMsg, "Network server startup failed with result: %d", result);
        log_main(errorMsg);
    }
    
    log_main("socket_main returning to GUI immediately");
}

// Stop the socket server manually
void stop_socket_server(void) {
    log_main("stop_socket_server called - stopping background threads and timer");
    
    if (g_hSocketThread == NULL && g_hMessageThread == NULL) {
        log_main("No server threads are running");
    } else {
        // Signal shutdown and cleanup
        cleanup_threads();
        log_main("Socket server threads stopped successfully");
    }
    
    // Stop the main thread timer
    stop_main_thread_timer();
    
    log_main("Socket server stopped successfully");
}

// 函数声明
void TransistorMacroWithLayers(const char* gateLayerName, const char* channelLayerName, const char* sourceLayerName);

// Thread-safe version of TransistorMacro - no GUI dialogs - Enhanced with layout info extraction
void TransistorMacro(void)
{
    // 使用默认图层名调用带参数的版本
    TransistorMacroWithLayers("Gate", "mos2", "Source");
}

// 增强版TransistorMacro函数，支持自定义图层名称
void TransistorMacroWithLayers(const char* gateLayerName, const char* channelLayerName, const char* sourceLayerName)
{
    log_main("TransistorMacroWithLayers called - starting enhanced thread-safe version");
    
    // 使用默认值如果参数为空
    const char* gateLayer_name = (gateLayerName && strlen(gateLayerName) > 0) ? gateLayerName : "Gate";
    const char* channelLayer_name = (channelLayerName && strlen(channelLayerName) > 0) ? channelLayerName : "mos2";
    const char* sourceLayer_name = (sourceLayerName && strlen(sourceLayerName) > 0) ? sourceLayerName : "Source";
    
    char paramMsg[200];
    sprintf(paramMsg, "Using layers: Gate=%s, Channel=%s, Source=%s", 
            gateLayer_name, channelLayer_name, sourceLayer_name);
    log_main(paramMsg);
    
    // 清空之前的结果
    memset(&g_lastResult, 0, sizeof(g_lastResult));
    g_lastResult.success = 0;
    strcpy(g_lastResult.error_message, "");
    strcpy(g_lastResult.custom_response, "");
    
    LPoint       gatePolygon[4], mos2Polygon[4], source1Polygon[4], source2Polygon[4];
    float          W, L;
    float        mos2_tolerance, contact_tolerance_w, contact_tolerance_l, gate_tolerance_w, gate_tolerance_l;
    
    try {
        log_main("Attempting to get visible cell...");
        LCell        Cell_Draw = LCell_GetVisible();
        
        if (Cell_Draw == NULL) {
            log_main("ERROR: LCell_GetVisible() returned NULL");
            log_main("Trying alternative method to get current cell...");
            
            // Try alternative method - get the first available cell
            // This might work better in threaded context
            LFile currentFile = LFile_GetVisible();
            if (currentFile != NULL) {
                log_main("Found visible file, trying to get its cells...");
                Cell_Draw = LCell_GetList(currentFile);  // Get first cell using LCell_GetList
                if (Cell_Draw != NULL) {
                    log_main("Successfully got first cell from visible file using LCell_GetList");
                } else {
                    log_main("ERROR: No cells found in visible file");
                    strcpy(g_lastResult.error_message, "No cells found in visible file");
                    return;
                }
            } else {
                log_main("ERROR: No visible file found either");
                strcpy(g_lastResult.error_message, "No visible file found");
                return;
            }
        } else {
            log_main("Successfully got visible cell using LCell_GetVisible()");
        }
        
        LFile        File_Draw = LCell_GetFile(Cell_Draw);
        if (File_Draw == NULL) {
            log_main("ERROR: Could not get file from cell");
            strcpy(g_lastResult.error_message, "Could not get file from cell");
            return;
        } else {
            log_main("Successfully got file from cell");
        }
        
        // 提取单元名称
        char cellNameBuffer[128];
        if (LCell_GetName(Cell_Draw, cellNameBuffer, sizeof(cellNameBuffer)) != NULL) {
            strncpy(g_lastResult.layout_data.cell_name, cellNameBuffer, sizeof(g_lastResult.layout_data.cell_name) - 1);
            g_lastResult.layout_data.cell_name[sizeof(g_lastResult.layout_data.cell_name) - 1] = '\0';
        } else {
            strcpy(g_lastResult.layout_data.cell_name, "Unknown");
        }
        
        // Check if required layers exist (no GUI dialog) - 使用可配置的图层名
        log_main("Checking for required layers...");
        
        LLayer gateLayer = LLayer_Find(File_Draw, gateLayer_name);
        LLayer mos2Layer = LLayer_Find(File_Draw, channelLayer_name);
        LLayer sourceLayer = LLayer_Find(File_Draw, sourceLayer_name);
        
        // 构建层信息字符串
        char layerInfo[512] = "";
        strcat(layerInfo, "Layers: ");
        
        if (gateLayer == NULL) {
            char msg[100];
            sprintf(msg, "ERROR: %s layer not found in layer map", gateLayer_name);
            log_main(msg);
            sprintf(msg, "%s(missing) ", gateLayer_name);
            strcat(layerInfo, msg);
        } else {
            char msg[100];
            sprintf(msg, "%s layer found", gateLayer_name);
            log_main(msg);
            sprintf(msg, "%s(ok) ", gateLayer_name);
            strcat(layerInfo, msg);
        }
        
        if (mos2Layer == NULL) {
            char msg[100];
            sprintf(msg, "ERROR: %s layer not found in layer map", channelLayer_name);
            log_main(msg);
            sprintf(msg, "%s(missing) ", channelLayer_name);
            strcat(layerInfo, msg);
        } else {
            char msg[100];
            sprintf(msg, "%s layer found", channelLayer_name);
            log_main(msg);
            sprintf(msg, "%s(ok) ", channelLayer_name);
            strcat(layerInfo, msg);
        }
        
        if (sourceLayer == NULL) {
            char msg[100];
            sprintf(msg, "ERROR: %s layer not found in layer map", sourceLayer_name);
            log_main(msg);
            sprintf(msg, "%s(missing) ", sourceLayer_name);
            strcat(layerInfo, msg);
        } else {
            char msg[100];
            sprintf(msg, "%s layer found", sourceLayer_name);
            log_main(msg);
            sprintf(msg, "%s(ok) ", sourceLayer_name);
            strcat(layerInfo, msg);
        }
        
        strcpy(g_lastResult.layout_data.layer_info, layerInfo);
        
        if (gateLayer == NULL || mos2Layer == NULL || sourceLayer == NULL) {
            char msg[200];
            sprintf(msg, "ERROR: One or more required layers (%s, %s, %s) not found in layer map", 
                    gateLayer_name, channelLayer_name, sourceLayer_name);
            log_main(msg);
            log_main("Please ensure these layers exist in your L-Edit layer configuration");
            strcpy(g_lastResult.error_message, "Required layers not found in layer map");
            return;
        }
        
        log_main("All required layers found, creating transistor geometry");
        
        // Use fixed position (0,0) instead of cursor position to avoid thread issues
        LPoint Translation = LPoint_Set(0, 0);
        
        // Use default values instead of GUI dialog - thread safe
        W = 45000.0f;  // Default width
        L = 2000.0f;   // Default length
        
        // 记录几何参数
        g_lastResult.layout_data.width = W;
        g_lastResult.layout_data.length = L;
        g_lastResult.layout_data.x_position = (float)Translation.x;
        g_lastResult.layout_data.y_position = (float)Translation.y;
        
        char paramMsg[200];
        sprintf(paramMsg, "Creating transistor with W=%.0f, L=%.0f at fixed position (%.0f, %.0f)", 
                W, L, (float)Translation.x, (float)Translation.y);
        log_main(paramMsg);
        
        // Calculate geometry
        mos2_tolerance = 2000;
        mos2Polygon[0] = LPoint_Set(-W / 2 + Translation.x, -(L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[1] = LPoint_Set(W / 2 + Translation.x, -(L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[2] = LPoint_Set(W / 2 + Translation.x, (L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[3] = LPoint_Set(-W / 2 + Translation.x, (L / 2 + mos2_tolerance) + Translation.y);
        
        gate_tolerance_w = 7500;
        gate_tolerance_l = 4000;
        gatePolygon[0] = LPoint_Set(-(W / 2+gate_tolerance_w) + Translation.x, -(L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[1] = LPoint_Set((W / 2 + gate_tolerance_w) + Translation.x, -(L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[2] = LPoint_Set((W / 2 + gate_tolerance_w) + Translation.x, (L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[3] = LPoint_Set(-(W / 2 + gate_tolerance_w) + Translation.x, (L / 2 + gate_tolerance_l) + Translation.y);
        
        contact_tolerance_w = 5000;
        contact_tolerance_l = 10000;
        source1Polygon[0] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) +(L/2+contact_tolerance_l/2) +Translation.y);
        source1Polygon[1] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source1Polygon[2] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source1Polygon[3] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        
        source2Polygon[0] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) -(L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[1] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[2] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[3] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        
        log_main("Creating polygon objects...");
        
        // Create the polygons using the layer objects we already found
        LPolygon_New(Cell_Draw, gateLayer, gatePolygon, 4);
        LPolygon_New(Cell_Draw, mos2Layer, mos2Polygon, 4);
        LPolygon_New(Cell_Draw, sourceLayer, source1Polygon, 4);
        LPolygon_New(Cell_Draw, sourceLayer, source2Polygon, 4);
        
        // 记录多边形数量
        g_lastResult.layout_data.polygon_count = 4;
        
        log_main("Polygons created successfully");
        
        // 添加时间戳
        SYSTEMTIME st;
        GetSystemTime(&st);
        sprintf(g_lastResult.layout_data.timestamp, "%04d-%02d-%02d %02d:%02d:%02d", 
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        
        // Update display - these should be thread-safe
        LCell_MakeVisible(Cell_Draw);
        LCell_HomeView(Cell_Draw);
        
        // 创建详细的响应信息
        sprintf(g_lastResult.custom_response, 
                "Transistor created successfully! Cell: %s, W=%.0f, L=%.0f, Position=(%.0f,%.0f), Polygons: %d, Time: %s",
                g_lastResult.layout_data.cell_name,
                g_lastResult.layout_data.width,
                g_lastResult.layout_data.length,
                g_lastResult.layout_data.x_position,
                g_lastResult.layout_data.y_position,
                g_lastResult.layout_data.polygon_count,
                g_lastResult.layout_data.timestamp);
        
        g_lastResult.success = 1;  // 标记成功
        log_main("TransistorMacro completed successfully");
        
    } catch (...) {
        log_main("EXCEPTION: Error occurred in TransistorMacro");
        g_lastResult.success = 0;
        strcpy(g_lastResult.error_message, "Exception occurred during transistor creation");
    }
}

// 仅当当前位置就是 layer=...; 才跳过；顺带去掉空白与'('
static const char* skip_layer_segment(const char* s) {
    if (!s) return "";
    const char* p = s;

    // 去掉前导空格或括号
    while (*p == ' ' || *p == '\t' || *p == '(') ++p;

    // 判断是否是 layer 段
    if (_strnicmp(p, "layer", 5) == 0) {
        p += 5; // 跳过 layer

        // 允许有空格
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '=') ++p;

        // 跳过空格
        while (*p == ' ' || *p == '\t') ++p;

        // 跳过层名（直到分号、空格或逗号）
        while (*p && *p != ';' && *p != ',' && *p != ' ' && *p != ')') ++p;

        // 跳过分号
        if (*p == ';') ++p;

        // 去掉后续空格
        while (*p == ' ' || *p == '\t') ++p;
    }

    return p;
}


// 把指针放到关键字(如 "polygon")之后的第一个坐标上
static const char* after_keyword_to_points(const char* p, const char* kw) {
    if (!p) return "";
    while (*p==' '||*p=='\t'||*p=='(') ++p;           // 允许形如 "(polygon ..."
    size_t n = strlen(kw);
    if (_strnicmp(p, kw, n)==0) {
        p += n;
    }
    while (*p==' '||*p=='\t') ++p;                    // 到达坐标或参数
    return p;
}

// 通用：读取一系列 "x y" 点，允许分号/空格/逗号混用
static int parse_point_list_strict(const char* p, LPoint* out, int maxPts) {
    if (!p) return 0;
    int cnt = 0;
    while (*p && cnt < maxPts) {
        // 结束符（boolexpr里的下一个图元 或 右括号）
        if (*p=='/' || *p==')') break;

        // 跳过分隔符
        while (*p==';' || *p==',' || isspace((unsigned char)*p)) ++p;

        // 解析 x
        char* endp = NULL;
        double x = strtod(p, &endp);
        if (endp==p) break;                           // 再也读不到数字了
        p = endp;
        while (*p==',' || isspace((unsigned char)*p)) ++p;

        // 解析 y
        double y = strtod(p, &endp);
        if (endp==p) break;
        p = endp;

        out[cnt++] = LPoint_Set((LCoord)x, (LCoord)y);

        // 跳过当前点后的分隔符，继续下一对
        while (*p==';' || *p==',' || isspace((unsigned char)*p)) ++p;
    }
    return cnt;
}


// 通用：读取两个整数（如中心点坐标）
static void parse_two_ints(const char* s, int& a, int& b) {
    if (!s) return;
    int x=0,y=0;
    if (sscanf(s,"%d %d",&x,&y)==2) { a=x; b=y; }
}

void PolygonMacro(const char* params) {
    try {
        log_main("PolygonMacro called");
        memset(&g_lastResult, 0, sizeof(g_lastResult));

        // ---- 1) 获取当前可见单元 ----
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // ---- 2) 获取 layer 名称 ----
        char layerName[64] = "Source";
        if (params && strstr(params, "layer")) {
            const char* lp = strstr(params, "layer");
            const char* eq = strchr(lp, '=');
            if (eq) {
                ++eq; while (*eq == ' ') ++eq;
                int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }

        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 3) 关键：坐标解析（修复首点丢失） ----
        const char* p = params ? params : "";
        p = skip_layer_segment(p);                  // 跳过 layer= 段（如果存在）
        p = after_keyword_to_points(p, "polygon");  // 定位到第一个点坐标

        LPoint pts[200];
        int n = parse_point_list_strict(p, pts, 200);
        if (n < 3) {
            char buf[128]; sprintf(buf, "ERROR: polygon needs >=3 points (parsed %d)", n);
            log_main(buf);
            return;
        }

        // ---- 4) 创建多边形 ----
        LPolygon_New(cell, layer, pts, n);
        char buf[128];
        sprintf(buf, "Polygon created (%d pts)", n);
        log_main(buf);

        LCell_MakeVisible(cell);
        LCell_HomeView(cell);
    }
    catch (...) {
        log_main("EXCEPTION in PolygonMacro");
    }
}


void PathMacro(const char* params) {
    try {
        log_main("PathMacro called");
        memset(&g_lastResult, 0, sizeof(g_lastResult));

        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // ---- 解析 layer / width ----
        char layerName[64] = "Source";
        int width = 100;

        if (params && strstr(params, "layer")) {
            const char* lp = strstr(params, "layer");
            const char* eq = strchr(lp, '=');
            if (eq) { ++eq; while (*eq == ' ') ++eq; int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }
        if (params && strstr(params, "width")) {
            const char* wp = strstr(params, "width");
            const char* eq = strchr(wp, '=');
            if (eq) { ++eq; while (*eq == ' ') ++eq;
                int w; if (sscanf(eq, "%d", &w) == 1 && w > 0) width = w;
            }
        }

        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 关键：对齐到 "path" 后第一个坐标，并严格解析 ----
        const char* p = skip_layer_segment(params ? params : "");
        p = after_keyword_to_points(p, "path");

        LPoint pts[200];
        int n = parse_point_list_strict(p, pts, 200);   // 不会再丢首点
        if (n < 2) { log_main("ERROR: need >=2 points for path"); return; }

        // ---- 生成 ----
        LWireConfig cfg; memset(&cfg, 0, sizeof(cfg)); cfg.width = width;
        LWire_New(cell, layer, &cfg, 0, pts, n);

        char buf[128]; sprintf(buf, "Path created (%d pts, w=%d)", n, width); log_main(buf);
        LCell_MakeVisible(cell); LCell_HomeView(cell);
    } catch (...) {
        log_main("EXCEPTION in PathMacro");
    }
}


// 简单的JSON解析辅助函数
static const char* find_json_string(const char* json, const char* key) {
    if (!json || !key) return NULL;
    
    char search_key[128];
    snprintf(search_key, sizeof(search_key), "\"%s\"", key);
    
    const char* pos = strstr(json, search_key);
    if (!pos) return NULL;
    
    // 找到 : 后的引号
    pos = strchr(pos, ':');
    if (!pos) return NULL;
    
    while (*pos && (*pos == ':' || *pos == ' ' || *pos == '\t')) pos++;
    
    if (*pos == '"') {
        return pos + 1;  // 返回引号后的内容
    }
    
    return NULL;
}

static int find_json_number(const char* json, const char* key) {
    if (!json || !key) return 0;
    
    char search_key[128];
    snprintf(search_key, sizeof(search_key), "\"%s\"", key);
    
    const char* pos = strstr(json, search_key);
    if (!pos) return 0;
    
    pos = strchr(pos, ':');
    if (!pos) return 0;
    
    return atoi(pos + 1);
}

static int find_json_bool(const char* json, const char* key) {
    if (!json || !key) return 0;
    
    char search_key[128];
    snprintf(search_key, sizeof(search_key), "\"%s\"", key);
    
    const char* pos = strstr(json, search_key);
    if (!pos) return 0;
    
    pos = strchr(pos, ':');
    if (!pos) return 0;
    
    while (*pos && (*pos == ':' || *pos == ' ' || *pos == '\t')) pos++;
    
    return (strncmp(pos, "true", 4) == 0) ? 1 : 0;
}

void TextMacro(const char* params) {
    try {
        log_main("TextMacro called - converting text to polygons via Python");
        memset(&g_lastResult, 0, sizeof(g_lastResult));
        
        // 解析参数: text=<文字>; mag=<大小>; x=<x坐标>; y=<y坐标>; layer=<层名>
        char text_content[256] = "TEXT";
        float mag = 30.0;
        float x_offset = 0.0;
        float y_offset = 0.0;
        char layer_name[64] = "Source";
        
        if (params && strlen(params) > 0) {
            // 解析text参数
            const char* text_pos = strstr(params, "text=");
            if (text_pos) {
                text_pos += 5;  // 跳过 "text="
                while (*text_pos == ' ') text_pos++;
                
                int i = 0;
                while (*text_pos && *text_pos != ';' && *text_pos != ',' && i < 255) {
                    text_content[i++] = *text_pos++;
                }
                text_content[i] = '\0';
                
                // 去掉尾部空格
                while (i > 0 && text_content[i-1] == ' ') {
                    text_content[--i] = '\0';
                }
            }
            
            // 解析mag参数
            const char* mag_pos = strstr(params, "mag=");
            if (mag_pos) {
                sscanf(mag_pos + 4, "%f", &mag);
            }
            
            // 解析x参数
            const char* x_pos = strstr(params, "x=");
            if (x_pos) {
                sscanf(x_pos + 2, "%f", &x_offset);
            }
            
            // 解析y参数
            const char* y_pos = strstr(params, "y=");
            if (y_pos) {
                sscanf(y_pos + 2, "%f", &y_offset);
            }
            
            // 解析layer参数
            const char* layer_pos = strstr(params, "layer=");
            if (layer_pos) {
                layer_pos += 6;
                while (*layer_pos == ' ') layer_pos++;
                
                int i = 0;
                while (*layer_pos && *layer_pos != ';' && *layer_pos != ',' && i < 63) {
                    layer_name[i++] = *layer_pos++;
                }
                layer_name[i] = '\0';
                
                while (i > 0 && layer_name[i-1] == ' ') {
                    layer_name[--i] = '\0';
                }
            }
        }
        
        char debug_msg[512];
        sprintf(debug_msg, "TextMacro parameters: text='%s', mag=%.1f, offset=(%.0f, %.0f), layer=%s",
                text_content, mag, x_offset, y_offset, layer_name);
        log_main(debug_msg);
        
        // 构建Python命令 - 使用cd切换目录后执行，避免路径编码问题
        char python_cmd[2048];
        const char* script_dir = "D:\\leditapi\\ledit_python";
        const char* venv_python_rel = "text_venv\\Scripts\\python.exe";
        const char* script_name = "text_to_polygon.py";
        
        // 检查虚拟环境是否存在（使用完整路径检查）
        char venv_python_full[512];
        sprintf(venv_python_full, "%s\\%s", script_dir, venv_python_rel);
        
        // 转换为宽字符以正确处理中文路径（使用系统默认代码页）
        wchar_t venv_python_wide[512];
        MultiByteToWideChar(CP_ACP, 0, venv_python_full, -1, venv_python_wide, 512);
        DWORD attr = GetFileAttributesW(venv_python_wide);
        int use_venv = (attr != INVALID_FILE_ATTRIBUTES);
        
        sprintf(debug_msg, "Checking venv path: %s (exists: %s)", venv_python_full, use_venv ? "YES" : "NO");
        log_main(debug_msg);
        
        if (!use_venv) {
            log_main("WARNING: Virtual environment not found, using system Python");
            // 切换到脚本目录后执行，使用相对路径
            sprintf(python_cmd, "cd /d \"%s\" && python %s \"%s\" %.1f %.0f %.0f \"%s\" 2>&1",
                    script_dir, script_name, text_content, mag, x_offset, y_offset, layer_name);
        } else {
            log_main("Using Python virtual environment");
            // 切换到脚本目录后执行，使用相对路径
            sprintf(python_cmd, "cd /d \"%s\" && %s %s \"%s\" %.1f %.0f %.0f \"%s\" 2>&1",
                    script_dir, venv_python_rel, script_name, text_content, mag, x_offset, y_offset, layer_name);
        }
        
        sprintf(debug_msg, "Executing Python command: %s", python_cmd);
        log_main(debug_msg);
        
        // 执行Python脚本并捕获输出
        FILE* pipe = _popen(python_cmd, "r");
        if (!pipe) {
            log_main("ERROR: Failed to execute Python script");
            strcpy(g_lastResult.error_message, "Failed to execute Python script");
            return;
        }
        
        // 读取Python输出（JSON格式）
        char json_output[16384] = "";
        char line[1024];
        while (fgets(line, sizeof(line), pipe)) {
            if (strlen(json_output) + strlen(line) < sizeof(json_output) - 1) {
                strcat(json_output, line);
            }
        }
        
        int exit_code = _pclose(pipe);
        
        sprintf(debug_msg, "Python script exit code: %d, output length: %d", exit_code, (int)strlen(json_output));
        log_main(debug_msg);
        
        if (strlen(json_output) == 0) {
            log_main("ERROR: No output from Python script");
            strcpy(g_lastResult.error_message, "No output from Python script");
            return;
        }
        
        // 记录Python的原始输出到日志（用于调试）
        log_main("=== Python Script Output START ===");
        log_main(json_output);
        log_main("=== Python Script Output END ===");
        
        // 解析JSON输出
        int success = find_json_bool(json_output, "success");
        int polygon_count = find_json_number(json_output, "polygon_count");
        
        sprintf(debug_msg, "JSON parse result: success=%d, polygon_count=%d", success, polygon_count);
        log_main(debug_msg);
        
        if (!success || polygon_count == 0) {
            log_main("ERROR: Python script failed to generate polygons");
            
            // 尝试从Python输出中提取错误信息
            const char* error_pos = strstr(json_output, "\"error\"");
            if (error_pos) {
                const char* error_msg = find_json_string(json_output, "error");
                if (error_msg) {
                    // 复制错误信息（直到下一个引号）
                    int i = 0;
                    while (error_msg[i] && error_msg[i] != '"' && i < 200) {
                        g_lastResult.error_message[i] = error_msg[i];
                        i++;
                    }
                    g_lastResult.error_message[i] = '\0';
                    sprintf(debug_msg, "Python error: %s", g_lastResult.error_message);
                    log_main(debug_msg);
                } else {
                    strcpy(g_lastResult.error_message, "Failed to generate polygons from text");
                }
            } else {
                strcpy(g_lastResult.error_message, "Failed to generate polygons from text");
            }
            return;
        }
        
        // 查找polygons数组
        const char* polygons_start = strstr(json_output, "\"polygons\"");
        if (!polygons_start) {
            log_main("ERROR: Cannot find polygons array in JSON output");
            strcpy(g_lastResult.error_message, "Invalid JSON output format");
            return;
        }
        
        // 获取当前单元
        LCell cell = LCell_GetVisible();
        if (!cell) {
            log_main("ERROR: No visible cell");
            strcpy(g_lastResult.error_message, "No visible cell");
            return;
        }
        
        LFile file = LCell_GetFile(cell);
        if (!file) {
            log_main("ERROR: Could not get file");
            strcpy(g_lastResult.error_message, "Could not get file");
            return;
        }
        
        // 查找或创建目标图层
        LLayer target_layer = LLayer_Find(file, layer_name);
        if (!target_layer) {
            LLayer_New(file, NULL, layer_name);
            target_layer = LLayer_Find(file, layer_name);
        }
        if (!target_layer) {
            log_main("ERROR: Cannot create target layer");
            strcpy(g_lastResult.error_message, "Cannot create target layer");
            return;
        }
        
        // 解析并绘制每个多边形
        int polygons_created = 0;
        const char* poly_pos = strstr(polygons_start, "\"points\"");
        
        while (poly_pos != NULL) {
            // 查找points数组的开始 [
            const char* points_start = strchr(poly_pos, '[');
            if (!points_start) break;
            points_start++;
            
            // 查找points数组的结束 ] - 需要匹配括号层级
            const char* points_end = points_start;
            int bracket_depth = 1;  // 已经进入第一层 [
            while (*points_end && bracket_depth > 0) {
                if (*points_end == '[') bracket_depth++;
                else if (*points_end == ']') bracket_depth--;
                if (bracket_depth > 0) points_end++;
            }
            if (bracket_depth != 0 || !*points_end) break;  // 没有找到匹配的 ]
            
            // 解析点坐标
            LPoint pts[200];
            int pt_count = 0;
            const char* p = points_start;
            
            while (p < points_end && pt_count < 200) {
                // 跳过空白和 [
                while (*p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '[')) p++;
                if (p >= points_end) break;
                
                // 读取x坐标
                double x = 0, y = 0;
                if (sscanf(p, "%lf", &x) == 1) {
                    // 跳过x坐标
                    while (*p && *p != ',' && *p != ']') p++;
                    if (*p == ',') p++;
                    
                    // 读取y坐标
                    while (*p && (*p == ' ' || *p == '\t')) p++;
                    if (sscanf(p, "%lf", &y) == 1) {
                        // Python输出的坐标是纳米单位，L-Edit使用微米，需要除以1000，并应用偏移量
                        pts[pt_count++] = LPoint_Set((LCoord)(x / 1000.0 + x_offset), (LCoord)(y / 1000.0 + y_offset));
                        
                        // 跳到下一个点或结束
                        while (*p && *p != ']' && *p != '[') p++;
                        if (*p == ']') p++;
                        if (*p == ',') p++;
                    }
                }
            }
            
            if (pt_count >= 3) {
                LPolygon_New(cell, target_layer, pts, pt_count);
                polygons_created++;
                
                sprintf(debug_msg, "Created polygon %d with %d points", polygons_created, pt_count);
                log_main(debug_msg);
            }
            
            // 查找下一个多边形
            poly_pos = strstr(points_end, "\"points\"");
        }
        
        if (polygons_created > 0) {
            LCell_MakeVisible(cell);
            LCell_HomeView(cell);
            
            g_lastResult.success = 1;
            g_lastResult.layout_data.polygon_count = polygons_created;
            sprintf(g_lastResult.custom_response, 
                    "Text '%s' converted to %d polygons successfully (mag=%.1f, offset=(%.0f,%.0f))",
                    text_content, polygons_created, mag, x_offset, y_offset);
            log_main(g_lastResult.custom_response);
        } else {
            log_main("ERROR: No polygons were created");
            strcpy(g_lastResult.error_message, "No polygons were created");
        }
        
    } catch (...) {
        log_main("EXCEPTION in TextMacro");
        strcpy(g_lastResult.error_message, "Exception in TextMacro");
    }
}

void CircleMacro(const char* params) {
    try {
        log_main("CircleMacro called");
        memset(&g_lastResult, 0, sizeof(g_lastResult));

        // ---- 1) 获取当前 cell / file ----
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // ---- 2) 解析 layer ----
        char layerName[64] = "Source";
        if (params && strstr(params, "layer")) {
            const char* lp = strstr(params, "layer");
            const char* eq = strchr(lp, '=');
            if (eq) {
                ++eq; while (*eq == ' ') ++eq;
                int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }

        // ---- 3) 解析半径 ----
        double radius = 100;
        if (params && strstr(params, "radius")) {
            const char* rp = strstr(params, "radius");
            const char* eq = strchr(rp, '=');
            if (eq) sscanf(eq + 1, "%lf", &radius);
        }

        // ---- 4) 解析中心坐标 ----
        const char* p = skip_layer_segment(params ? params : "");
        p = after_keyword_to_points(p, "circle");

        LPoint centerPts[4];
        int n = parse_point_list_strict(p, centerPts, 4);
        if (n < 1) { log_main("ERROR: need center point for circle"); return; }

        LCoord cx = centerPts[0].x;
        LCoord cy = centerPts[0].y;

        // ---- 5) 查找/创建图层 ----
        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 6) 在本函数内直接生成 polygon 圆 ----
        const int N = 128;  // 边数，可调分辨率
        LPoint pts[N];
        for (int i = 0; i < N; ++i) {
            double theta = 2.0 * M_PI * i / N;
            LCoord x = (LCoord)llround((double)cx + radius * cos(theta));
            LCoord y = (LCoord)llround((double)cy + radius * sin(theta));
            pts[i] = LPoint_Set(x, y);
        }

        LPolygon_New(cell, layer, pts, N);
        char buf[160];
        sprintf(buf, "Polygonal circle created on %s center=(%ld,%ld) radius=%.0f N=%d",
                layerName, (long)cx, (long)cy, radius, N);
        log_main(buf);

        LCell_MakeVisible(cell);
        LCell_HomeView(cell);
    }
    catch (...) {
        log_main("EXCEPTION in CircleMacro");
    }
}



// 工具函数：把角度规范到 [0,360)
static double norm360(int a) {
    int t = a % 360;
    if (t < 0) t += 360;
    return static_cast<double>(t);
}

void PieWedgeMacro(const char* params) {
    try {
        log_main("PieWedgeMacro called");
        memset(&g_lastResult, 0, sizeof(g_lastResult));

        // ---- 1) 获取当前 cell / file ----
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // ---- 2) 解析 layer ----
        char layerName[64] = "Source";
        if (params && strstr(params, "layer")) {
            const char* lp = strstr(params, "layer");
            const char* eq = strchr(lp, '=');
            if (eq) {
                ++eq; while (*eq == ' ') ++eq;
                int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }

        // ---- 3) 解析半径与角度 ----
        double radius = 100;
        double a1_deg = 0, a2_deg = 90;
        if (params && strstr(params, "radius")) {
            const char* rp = strstr(params, "radius");
            const char* eq = strchr(rp, '=');
            if (eq) sscanf(eq + 1, "%lf", &radius);
        }
        if (params && strstr(params, "angle")) {
            const char* ap = strstr(params, "angle");
            const char* eq = strchr(ap, '=');
            if (eq) sscanf(eq + 1, "%lf %lf", &a1_deg, &a2_deg);
        }

        // ---- 4) 解析中心点 ----
        const char* p = skip_layer_segment(params ? params : "");
        p = after_keyword_to_points(p, "piewedge");

        LPoint centerPts[4];
        int n = parse_point_list_strict(p, centerPts, 4);
        if (n < 1) { log_main("ERROR: need center point for piewedge"); return; }

        LCoord cx = centerPts[0].x;
        LCoord cy = centerPts[0].y;

        // ---- 5) 获取 layer ----
        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 6) 生成 polygon 形式扇形 ----
        const int N = 128;
        double a1 = a1_deg * M_PI / 180.0;
        double a2 = a2_deg * M_PI / 180.0;
        double span = a2 - a1;
        if (span == 0) span = 2.0 * M_PI;

        int K = (int)std::max(2.0, ceil(fabs(span) / (2.0 * M_PI) * N));
        std::vector<LPoint> pts;
        pts.reserve(K + 3);
        pts.push_back(LPoint_Set(cx, cy));
        for (int i = 0; i <= K; ++i) {
            double t = a1 + span * i / K;
            LCoord x = (LCoord)llround((double)cx + radius * cos(t));
            LCoord y = (LCoord)llround((double)cy + radius * sin(t));
            pts.push_back(LPoint_Set(x, y));
        }

        LPolygon_New(cell, layer, pts.data(), (int)pts.size());

        char buf[200];
        sprintf(buf, "PieWedge polygon created on %s center=(%ld,%ld) r=%.0f angles=%.1f~%.1f (N=%d)",
                layerName, (long)cx, (long)cy, radius, a1_deg, a2_deg, K);
        log_main(buf);

        LCell_MakeVisible(cell);
        LCell_HomeView(cell);
    }
    catch (...) {
        log_main("EXCEPTION in PieWedgeMacro");
    }
}



void RingWedgeMacro(const char* params) {
    try {
        log_main("RingWedgeMacro called");
        memset(&g_lastResult, 0, sizeof(g_lastResult));

        // ---- 1) 获取 cell / file ----
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // ---- 2) layer ----
        char layerName[64] = "Source";
        if (params && strstr(params, "layer")) {
            const char* lp = strstr(params, "layer");
            const char* eq = strchr(lp, '=');
            if (eq) {
                ++eq; while (*eq == ' ') ++eq;
                int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }

        // ---- 3) 半径与角度 ----
        double r1 = 100, r2 = 200;
        double a1_deg = 0, a2_deg = 90;
        if (params && strstr(params, "radius")) {
            const char* rp = strstr(params, "radius");
            const char* eq = strchr(rp, '=');
            if (eq) sscanf(eq + 1, "%lf %lf", &r1, &r2);
        }
        if (params && strstr(params, "angle")) {
            const char* ap = strstr(params, "angle");
            const char* eq = strchr(ap, '=');
            if (eq) sscanf(eq + 1, "%lf %lf", &a1_deg, &a2_deg);
        }

        // ---- 4) 中心点 ----
        const char* p = skip_layer_segment(params ? params : "");
        p = after_keyword_to_points(p, "ringwedge");

        LPoint pts[4];
        int n = parse_point_list_strict(p, pts, 4);
        if (n < 1) { log_main("ERROR: need center point for ringwedge"); return; }
        LCoord cx = pts[0].x, cy = pts[0].y;

        // ---- 5) layer handle ----
        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 6) 生成 polygon 环形扇区 ----
        if (r2 < r1) std::swap(r1, r2);
        const int N = 128;
        double a1 = a1_deg * M_PI / 180.0;
        double a2 = a2_deg * M_PI / 180.0;
        double span = a2 - a1;
        if (span == 0) span = 2 * M_PI;
        int K = (int)std::max(2.0, ceil(fabs(span) / (2.0 * M_PI) * N));

        std::vector<LPoint> poly;
        poly.reserve(2 * (K + 1) + 2);

        // 外弧
        for (int i = 0; i <= K; ++i) {
            double t = a1 + span * i / K;
            LCoord x = (LCoord)llround((double)cx + r2 * cos(t));
            LCoord y = (LCoord)llround((double)cy + r2 * sin(t));
            poly.push_back(LPoint_Set(x, y));
        }
        // 内弧（反向）
        for (int i = 0; i <= K; ++i) {
            double t = a2 - span * i / K;
            LCoord x = (LCoord)llround((double)cx + r1 * cos(t));
            LCoord y = (LCoord)llround((double)cy + r1 * sin(t));
            poly.push_back(LPoint_Set(x, y));
        }

        LPolygon_New(cell, layer, poly.data(), (int)poly.size());
        char buf[200];
        sprintf(buf, "RingWedge polygon created on %s center=(%ld,%ld) r1=%.0f r2=%.0f angle=%.1f~%.1f N=%d",
                layerName, (long)cx, (long)cy, r1, r2, a1_deg, a2_deg, K);
        log_main(buf);

        LCell_MakeVisible(cell);
        LCell_HomeView(cell);
    }
    catch (...) {
        log_main("EXCEPTION in RingWedgeMacro");
    }
}

// 工具：确保存在一个临时层（若无则创建），返回层句柄
static LLayer EnsureTempLayer(LFile file, const char* name) {
    if (!file || !name) return NULL;
    LLayer layer = LLayer_Find(file, name);
    if (!layer) {
        if (LLayer_New(file, NULL, name) != LStatusOK) {
            char buf[128];
            sprintf(buf, "ERROR: failed to create temp layer %s", name);
            log_main(buf);
            return NULL;
        }
        layer = LLayer_Find(file, name);
    }
    return layer;
}

// ====== 工具：把一个对象表达式绘制到指定层（直接调用已有宏） ======
// 会把 expr 里的 layer=... 强制替换为目标临时层
static void DrawObjectToLayer(const char* expr, const char* tmpLayerName) {
    // 构造新的参数串：layer=TMP; <原其余参数>
    char buf[1024];
    if (strstr(expr, "layer=")) {
        const char* p = strstr(expr, "layer=");
        const char* semi = strchr(p, ';');
        if (semi) {
            std::snprintf(buf, sizeof(buf), "layer=%s;%s", tmpLayerName, semi + 1);
        } else {
            std::snprintf(buf, sizeof(buf), "layer=%s", tmpLayerName);
        }
    } else {
        std::snprintf(buf, sizeof(buf), "layer=%s;%s", tmpLayerName, expr);
    }

    // 判断类型并调用原宏（关键字不区分大小写可按需升级）
    if (strstr(expr, "polygon")) {
        PolygonMacro(buf);
    } else if (strstr(expr, "circle")) {
        CircleMacro(buf);
    } else if (strstr(expr, "path")) {
        PathMacro(buf);
    } else if (strstr(expr, "piewedge")) {
        PieWedgeMacro(buf);
    } else if (strstr(expr, "ringwedge")) {
        RingWedgeMacro(buf);
    } else {
        log_main("ERROR: Unknown object type in DrawObjectToLayer");
    }
}

// ====== 直接用对象布尔：从 lhs/rhs 两层取对象，做布尔，输出到新的普通层 ======
static std::string ApplyBoolOp(LCell cell, char op,
                               const std::string& lhsLayerName,
                               const std::string& rhsLayerName) {
    static int derivedCount = 0;
    char derivedName[64];
    std::sprintf(derivedName, "DERIVED_%d", ++derivedCount);

    LFile file = LCell_GetFile(cell);

    // 取左右操作数的层
    LLayer lhs = LLayer_Find(file, lhsLayerName.c_str());
    LLayer rhs = LLayer_Find(file, rhsLayerName.c_str());
    if (!lhs || !rhs) {
        log_main("ERROR: ApplyBoolOp: lhs/rhs layer not found");
        return lhsLayerName;
    }

    // 新建一个普通结果层（给后续继续参与运算）
    if (LLayer_New(file, NULL, derivedName) != LStatusOK) {
        char buf[160];
        std::sprintf(buf, "ERROR: create result layer failed: %s", derivedName);
        log_main(buf);
        return lhsLayerName;
    }
    LLayer derived = LLayer_Find(file, derivedName);
    if (!derived) {
        log_main("ERROR: cannot find result layer after creation");
        return lhsLayerName;
    }

    // 收集左右层上的对象
    std::vector<LObject> A, B;
    for (LObject o = LObject_GetList(cell, lhs); o; o = LObject_GetNext(o)) A.push_back(o);
    for (LObject o = LObject_GetList(cell, rhs); o; o = LObject_GetNext(o)) B.push_back(o);

    if (A.empty() && B.empty()) {
        log_main("WARN: both operands empty");
        return lhsLayerName;
    }

    // ====== 运算逻辑 ======
    if (op == '/') {
        // 视觉并集：复制，不做几何合并
        log_main("ApplyBoolOp: visual OR — copy all objects without Boolean union");

        // 复制 A 层对象到 derived
        for (size_t i = 0; i < A.size(); ++i) {
            LObject obj = A[i];
            LObject newObj = LObject_Copy(cell, derived, obj);  // ← 参数顺序纠正
            if (!newObj) log_main("WARN: copy A failed");
        }

        // 复制 B 层对象到 derived
        for (size_t i = 0; i < B.size(); ++i) {
            LObject obj = B[i];
            LObject newObj = LObject_Copy(cell, derived, obj);  // ← 参数顺序纠正
            if (!newObj) log_main("WARN: copy B failed");
        }

        int copied = 0;
        for (LObject o = LObject_GetList(cell, derived); o; o = LObject_GetNext(o))
            ++copied;

        char buf[256];
        std::sprintf(buf,
            "ApplyBoolOp visual OR: %s / %s -> %s (copied %d objs)",
            lhsLayerName.c_str(), rhsLayerName.c_str(), derivedName, copied);
        log_main(buf);

        return std::string(derivedName);
    }
    else {
        // ---------------------------
        // 真实布尔（交/差）
        // ---------------------------
        LBooleanOperation bop = (op == '+') ? LBoolOp_AND : LBoolOp_SUBTRACT;

        LStatus s = LCell_BooleanOperation(
            cell, bop,
            (LCoord)0,                                   // 公差
            A.data(), (unsigned int)A.size(),
            B.data(), (unsigned int)B.size(),
            derived, LFALSE                              // 结果写入 derived，不删除输入
        );

        if (s != LStatusOK) {
            log_main("ERROR: LCell_BooleanOperation failed");
            return lhsLayerName;
        }

        int created = 0;
        for (LObject o = LObject_GetList(cell, derived); o; o = LObject_GetNext(o)) ++created;

        char buf[256];
        std::sprintf(buf, "ApplyBoolOp Boolean: %s %c %s -> %s (created %d objs)",
                     lhsLayerName.c_str(), op, rhsLayerName.c_str(),
                     derivedName, created);
        log_main(buf);
    }

    return std::string(derivedName);
}

// ====== 解析一个“基本对象串”为临时层：返回绘制到的 TMP_x 层名 ======
static std::string ParseSingleObject(const char* objExpr, LCell cell, LFile file) {
    static int objCounter = 0;
    char tmpLayerName[64];
    std::sprintf(tmpLayerName, "TMP_%d", ++objCounter);

    LLayer tmpLayer = EnsureTempLayer(file, tmpLayerName);
    if (!tmpLayer) return "";

    // 关键：统一走 DrawObjectToLayer（它会把 layer= 改成 TMP_x）
    // 仅当对象串是“直接坐标起头”时，临时补上 "polygon "
    const char* p = objExpr ? objExpr : "";
    while (*p==' ' || *p=='\t' || *p=='(') ++p;

    if ((*p>='0' && *p<='9') || *p=='-' || *p=='+') {
        // e.g. "0 0; 500 0; ..." -> "polygon 0 0; 500 0; ..."
        char wrapped[1024];
        std::snprintf(wrapped, sizeof(wrapped), "polygon %s", objExpr);
        DrawObjectToLayer(wrapped, tmpLayerName);
    } else {
        DrawObjectToLayer(objExpr, tmpLayerName);
    }

    return std::string(tmpLayerName);
}

// 清理所有临时层（TMP_/DERIVED_ 开头）及其对象，保留指定层
static void DeleteTempLayers(LCell cell, LFile file, const char* keepLayerName /*=NULL*/) {
    std::vector<LLayer> toDelete;

    // 收集需要删除的层
    LLayer L = LLayer_GetList(file);
    while (L) {
        char name[128] = {0};
        if (LLayer_GetName(L, name, sizeof(name))) {
            int shouldDelete = 0;
            if ((strncmp(name, "TMP_", 4) == 0 || strncmp(name, "DERIVED_", 8) == 0)) {
                if (!keepLayerName || _stricmp(name, keepLayerName) != 0)
                    shouldDelete = 1;
            }
            if (shouldDelete)
                toDelete.push_back(L);
        }
        L = LLayer_GetNext(L);
    }

    // 遍历删除
    for (size_t i = 0; i < toDelete.size(); ++i) {
        LLayer layer = toDelete[i];
        char name[128] = {0};
        LLayer_GetName(layer, name, sizeof(name));
        log_main((std::string("Deleting temp layer ") + name).c_str());

        // 删除对象
        LObject obj = LObject_GetList(cell, layer);
        while (obj) {
            LObject next = LObject_GetNext(obj);
            LObject_Delete(cell, obj);
            obj = next;
        }

        LStatus s = LLayer_Delete(file, layer);
        char buf[256];
        sprintf(buf, "Deleted %s (status=%d)", name, s);
        log_main(buf);
    }
}


// ====== 递归解析表达式：支持括号嵌套与多运算，左结合 ======
// 输入 p：指向 '(' 后或表达式开头；遇到 ')' 或串末返回
// 返回：该子表达式结果所在层名（TMP_x 或 DERIVED_x）
// --- 用普通的辅助函数替代 lambda，以兼容更老的编译器
static inline void FlushOnce(std::vector<std::string>& layerStack, std::vector<char>& opStack, LCell cell) {
    if (layerStack.size() >= 2 && !opStack.empty()) {
        std::string rhs = layerStack.back(); layerStack.pop_back();
        std::string lhs = layerStack.back(); layerStack.pop_back();
        char op = opStack.back(); opStack.pop_back();
        std::string res = ApplyBoolOp(cell, op, lhs, rhs);
        layerStack.push_back(res);
    }
}

static std::string ParseExpr(const char*& p, LCell cell, LFile file) {
    std::vector<std::string> layerStack;
    std::vector<char>        opStack;

    while (*p) {
        while (std::isspace((unsigned char)*p)) ++p;
        if (!*p || *p == ')') break;

        if (*p == '(') {
            ++p; // 跳过 '('
            std::string inner = ParseExpr(p, cell, file);
            layerStack.push_back(inner);
            if (*p == ')') ++p; // 吃掉 ')'
            FlushOnce(layerStack, opStack, cell);
            continue;
        }

        if (*p == '/' || *p == '+' || *p == '-') {
            opStack.push_back(*p);
            ++p;
            continue;
        }

        // 解析一个对象串：读到下一个操作符或右括号为止（允许坐标里的负号）
        const char* start = p;
        while (*p && *p != ')') {
            // 真正的二元运算符
            if (*p == '/' || *p == '+') break;

            if (*p == '-') {
                // 如果 '-' 后面紧跟数字或小数点，视为坐标的单目负号；否则当成二元运算符
                char next = *(p + 1);
                if (!((next >= '0' && next <= '9') || next == '.')) {
                    break;  // 这里的 '-' 是运算符
                }
                // 否则是 -500 这样的数值前缀，继续往后扫
            }

            ++p;
        }

        std::string objStr(start, p - start);
        // 去掉对象串两端空白
        size_t a = objStr.find_first_not_of(" \t\r\n");
        size_t b = objStr.find_last_not_of(" \t\r\n");
        if (a == std::string::npos) {
            log_main("WARN: empty object string");
        } else {
            objStr = objStr.substr(a, b - a + 1);
            std::string tmpLayerName = ParseSingleObject(objStr.c_str(), cell, file);
            if (!tmpLayerName.empty()) {
                layerStack.push_back(tmpLayerName);
                FlushOnce(layerStack, opStack, cell);
            } else {
                log_main("ERROR: ParseSingleObject failed");
            }
        }
    }

    // 把剩余操作都做掉（左结合）
    while (layerStack.size() >= 2 && !opStack.empty()) {
        std::string rhs = layerStack.back(); layerStack.pop_back();
        std::string lhs = layerStack.back(); layerStack.pop_back();
        char op = opStack.back(); opStack.pop_back();
        std::string res = ApplyBoolOp(cell, op, lhs, rhs);
        layerStack.push_back(res);
    }

    if (layerStack.empty()) return "";
    return layerStack.back();
}

// ====== 顶层宏：boolexpr ======
void BoolExprMacro(const char* params) {
    try {
        log_main("BoolExprMacro called (Derived Layer)");

        std::memset(&g_lastResult, 0, sizeof(g_lastResult));
        g_lastResult.success = 0;

        // 1) 获取当前 cell / file
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file"); return; }

        // 2) 清除上轮的临时层
        DeleteTempLayers(cell, file, NULL);

        // 3) 解析目标层名
        char targetLayerName[64] = "Source";
        if (params && std::strstr(params, "layer")) {
            const char* pos = std::strstr(params, "layer");
            const char* eq  = std::strchr(pos, '=');
            if (eq) {
                ++eq; while (*eq == ' ') ++eq;
                int i = 0;
                while (*eq && *eq != ';' && *eq != ' ' && *eq != ',' && i < 63)
                    targetLayerName[i++] = *eq++;
                targetLayerName[i] = '\0';
            }
        }

        LLayer targetLayer = LLayer_Find(file, targetLayerName);
        if (!targetLayer) {
            if (LLayer_New(file, NULL, targetLayerName) != LStatusOK) {
                log_main("ERROR: cannot create target layer");
                return;
            }
            targetLayer = LLayer_Find(file, targetLayerName);
            if (!targetLayer) { log_main("ERROR: created target layer not found"); return; }
        }

        // 4) 找 '('
        const char* p = std::strchr(params ? params : "", '(');
        if (!p) { log_main("ERROR: No expression found"); return; }

        // ================================
        // 5) 改造点：对齐 polygon 子命令
        // ================================
        // 我们给 ParseExpr 的字符串进行轻度预处理
        // 使它的子宏参数能正确从坐标开头解析，不再丢第一个点。
        std::string preprocessed = p;
        // 去除前导空格与(
        while (!preprocessed.empty() &&
              (preprocessed[0]==' '||preprocessed[0]=='\t'||preprocessed[0]=='('))
            preprocessed.erase(0,1);

        // 为了保险，我们在 ParseExpr 内部再调用 after_keyword_to_points()
        const char* exprStart = after_keyword_to_points(preprocessed.c_str(), "polygon");

        // 6) 调用 ParseExpr（它内部再去判断 polygon/circle/...）
        std::string resultLayerName = ParseExpr(exprStart, cell, file);
        if (resultLayerName.empty()) { log_main("ERROR: ParseExpr returned empty"); return; }

        LLayer resultLayer = LLayer_Find(file, resultLayerName.c_str());
        if (!resultLayer) { log_main("ERROR: Result layer not found"); return; }

        // 7) 把结果对象搬到目标层
        LObject curr = LObject_GetList(cell, resultLayer);
        int moved = 0;
        while (curr) {
            LObject next = LObject_GetNext(curr);
            LObject_ChangeLayer(cell, curr, targetLayer);
            moved++;
            curr = next;
        }

        LCell_MakeVisible(cell);
        LCell_HomeView(cell);

        g_lastResult.success = 1;
        g_lastResult.layout_data.polygon_count = moved;

        char buf[256];
        std::sprintf(buf, "BoolExprMacro done: %d objects moved to %s", moved, targetLayerName);
        log_main(buf);
        std::strncpy(g_lastResult.custom_response, buf, sizeof(g_lastResult.custom_response)-1);

        // 8) 清除临时层（保护目标层）
        DeleteTempLayers(cell, file, targetLayerName);
    }
    catch (...) {
        log_main("EXCEPTION in BoolExprMacro");
        g_lastResult.success = 0;
        std::strcpy(g_lastResult.error_message, "Exception in BoolExprMacro");
    }
}

// ====== 阵列功能宏：ArrayMacro ======
void ArrayMacro(const char* params) {
    try {
        log_main("========== [ArrayMacro] START ==========");

        if (!params || !*params) {
            log_main("ERROR: No parameters given.");
            return;
        }

        // ---- 1) 解析 layer 名称 ----
        char layerName[64] = "Source";
        const char* lp = strstr(params, "layer");
        if (lp) {
            const char* eq = strchr(lp, '=');
            if (eq) {
                ++eq; while (*eq==' '||*eq=='\t') ++eq;
                int i=0;
                while (*eq && *eq!=';' && *eq!=' ' && *eq!=',' && i<63) layerName[i++] = *eq++;
                layerName[i] = '\0';
            }
        }

        // ---- 2) 检测图形类型 ----
        const char* shape = NULL;
        if ((shape = strstr(params, "polygon"))) {
            log_main("Detected shape: polygon");
        } else if ((shape = strstr(params, "path"))) {
            log_main("Detected shape: path");
        } else if ((shape = strstr(params, "piewedge"))) {
            log_main("Detected shape: piewedge");
        } else if ((shape = strstr(params, "ringwedge"))) {
            log_main("Detected shape: ringwedge");
        } else if ((shape = strstr(params, "circle"))) {
            log_main("Detected shape: circle");
        } else if ((shape = strstr(params, "text"))) {
            log_main("Detected shape: text");
        } else {
            log_main("ERROR: No valid shape keyword found.");
            return;
        }

        // ---- 3) 分离“几何段(geomPart)”与“阵列段(arrPart)” ----
        const char* tail = strstr(params, "nx");
        if (!tail) tail = strstr(params, "ny");
        if (!tail) tail = params + strlen(params);

        std::string geomPart(shape, tail);
        std::string arrPart(tail);

        // ---- 4) 解析阵列参数 ----
        int nx = 1, ny = 1;
        double dx = 0, dy = 0;
        const char* s;
        if ((s = strstr(arrPart.c_str(), "nx"))) sscanf(s, "nx = %d", &nx);
        if ((s = strstr(arrPart.c_str(), "dx"))) sscanf(s, "dx = %lf", &dx);
        if ((s = strstr(arrPart.c_str(), "ny"))) sscanf(s, "ny = %d", &ny);
        if ((s = strstr(arrPart.c_str(), "dy"))) sscanf(s, "dy = %lf", &dy);
        if (nx < 1) nx = 1;
        if (ny < 1) ny = 1;

        char logbuf[256];
        sprintf(logbuf, "Layer=%s, nx=%d, dx=%.2f, ny=%d, dy=%.2f", layerName, nx, dx, ny, dy);
        log_main(logbuf);

        // ---- 5) 获取 cell/file 与目标层 ----
        LCell cell = LCell_GetVisible();
        if (!cell) { log_main("ERROR: No visible cell"); return; }
        LFile file = LCell_GetFile(cell);
        if (!file) { log_main("ERROR: Could not get file from cell"); return; }
        LLayer layer = LLayer_Find(file, layerName);
        if (!layer) { LLayer_New(file, NULL, layerName); layer = LLayer_Find(file, layerName); }
        if (!layer) { log_main("ERROR: cannot find layer"); return; }

        // ---- 6) 阵列生成 ----
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                const double ox = ix * dx;
                const double oy = iy * dy;

                if (strstr(shape, "polygon")) {
                    LPoint pts[512];
                    const char* pcoords = after_keyword_to_points(geomPart.c_str(), "polygon");
                    int count = parse_point_list_strict(pcoords, pts, 512);
                    if (count <= 0) { log_main("ERROR: parse polygon points failed"); return; }
                    for (int k=0; k<count; ++k) { pts[k].x += (int)ox; pts[k].y += (int)oy; }
                    LPolygon_New(cell, layer, pts, count);
                }
                else if (strstr(shape, "path")) {
                    int width = 100;
                    if ((s = strstr(geomPart.c_str(), "width"))) {
                        const char* eq = strchr(s, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; int w; if (sscanf(eq, "%d", &w)==1 && w>0) width = w; }
                    }
                    const char* pcoords = geomPart.c_str();
                    const char* wpos = strstr(pcoords, "width");
                    if (wpos) {
                        const char* semi = strchr(wpos, ';');
                        const char* comma = strchr(wpos, ',');
                        const char* sep = NULL;
                        if (semi && comma) sep = (semi < comma ? semi : comma);
                        else if (semi) sep = semi;
                        else if (comma) sep = comma;
                        if (sep) pcoords = sep + 1;
                        else pcoords = wpos + strlen("width");
                    }
                    while (*pcoords && !isdigit((unsigned char)*pcoords) && *pcoords!='-' && *pcoords!='+') ++pcoords;

                    LPoint pts[512];
                    int count = parse_point_list_strict(pcoords, pts, 512);
                    if (count < 2) { log_main("ERROR: need >=2 points for path"); return; }
                    for (int k=0; k<count; ++k) { pts[k].x += (int)ox; pts[k].y += (int)oy; }

                    std::string cmd;
                    char head[128];
                    sprintf(head, "layer=%s; path", layerName);
                    cmd.append(head);
                    for (int k=0; k<count; ++k) {
                        char seg[64];
                        sprintf(seg, " %d %d,", (int)pts[k].x, (int)pts[k].y);
                        cmd.append(seg);
                    }
                    if (!cmd.empty()) cmd[cmd.size()-1] = ';';
                    char tail[64];
                    sprintf(tail, " width=%d;", width);
                    cmd.append(tail);
                    PathMacro(cmd.c_str());
                }
                else if (strstr(shape, "circle")) {
                    int cx = 0, cy = 0; double r = 0;
                    const char* p = after_keyword_to_points(geomPart.c_str(), "circle");
                    sscanf(p, "%d %d", &cx, &cy);
                    const char* rp = strstr(geomPart.c_str(), "radius");
                    if (rp) {
                        const char* eq = strchr(rp, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; sscanf(eq, "%lf", &r); }
                    }
                    cx += (int)ox; cy += (int)oy;
                    char sub[256];
                    sprintf(sub, "layer=%s; circle %d %d, radius=%g;", layerName, cx, cy, r);
                    CircleMacro(sub);
                }
                else if (strstr(shape, "piewedge")) {
                    int cx = 0, cy = 0; double r = 0, a1 = 0, a2 = 0;
                    const char* p = after_keyword_to_points(geomPart.c_str(), "piewedge");
                    sscanf(p, "%d %d", &cx, &cy);
                    const char* rp = strstr(geomPart.c_str(), "radius");
                    if (rp) {
                        const char* eq = strchr(rp, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; sscanf(eq, "%lf", &r); }
                    }
                    const char* ap = strstr(geomPart.c_str(), "angle");
                    if (ap) {
                        const char* eq = strchr(ap, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; sscanf(eq, "%lf %lf", &a1, &a2); }
                    }
                    cx += (int)ox; cy += (int)oy;
                    char sub[256];
                    sprintf(sub, "layer=%s; piewedge %d %d; radius=%g; angle=%g %g;", layerName, cx, cy, r, a1, a2);
                    PieWedgeMacro(sub);
                }
                else if (strstr(shape, "ringwedge")) {
                    int cx = 0, cy = 0; double r1 = 0, r2 = 0, a1 = 0, a2 = 0;
                    const char* p = after_keyword_to_points(geomPart.c_str(), "ringwedge");
                    sscanf(p, "%d %d", &cx, &cy);
                    const char* rp = strstr(geomPart.c_str(), "radius");
                    if (rp) {
                        const char* eq = strchr(rp, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; sscanf(eq, "%lf %lf", &r1, &r2); }
                    }
                    const char* ap = strstr(geomPart.c_str(), "angle");
                    if (ap) {
                        const char* eq = strchr(ap, '=');
                        if (eq) { ++eq; while (*eq==' ') ++eq; sscanf(eq, "%lf %lf", &a1, &a2); }
                    }
                    cx += (int)ox; cy += (int)oy;
                    char sub[256];
                    sprintf(sub, "layer=%s; ringwedge %d %d; radius=%g %g; angle=%g %g;", layerName, cx, cy, r1, r2, a1, a2);
                    RingWedgeMacro(sub);
                }
                else if (strstr(shape, "text")) {
                    // 解析text相关参数
                    char text_content[256] = "TEXT";
                    float mag = 30.0;
                    float base_x = 0.0, base_y = 0.0;
                    float text_offset_x = 0.0, text_offset_y = 0.0;
                    
                    // 提取text内容
                    const char* text_pos = strstr(geomPart.c_str(), "text=");
                    if (text_pos) {
                        text_pos += 5;  // 跳过 "text="
                        while (*text_pos == ' ') text_pos++;
                        
                        int i = 0;
                        while (*text_pos && *text_pos != ';' && *text_pos != ',' && i < 255) {
                            text_content[i++] = *text_pos++;
                        }
                        text_content[i] = '\0';
                        
                        // 去掉尾部空格
                        while (i > 0 && text_content[i-1] == ' ') {
                            text_content[--i] = '\0';
                        }
                    }
                    
                    // 提取mag参数
                    const char* mag_pos = strstr(geomPart.c_str(), "mag=");
                    if (mag_pos) {
                        sscanf(mag_pos + 4, "%f", &mag);
                    }
                    
                    // 提取x=偏移参数
                    const char* x_pos = strstr(geomPart.c_str(), "x=");
                    if (x_pos) {
                        sscanf(x_pos + 2, "%f", &text_offset_x);
                    }
                    
                    // 提取y=偏移参数
                    const char* y_pos = strstr(geomPart.c_str(), "y=");
                    if (y_pos) {
                        sscanf(y_pos + 2, "%f", &text_offset_y);
                    }
                    
                    // 提取基准坐标
                    const char* p = after_keyword_to_points(geomPart.c_str(), "text");
                    if (p && sscanf(p, "%f %f", &base_x, &base_y) == 2) {
                        // 成功解析坐标
                    }
                    
                    // 计算当前阵列位置的坐标偏移：基准坐标 + text自身偏移 + 阵列偏移
                    float final_x = base_x + text_offset_x + ox;
                    float final_y = base_y + text_offset_y + oy;
                    
                    // 构建TextMacro参数字符串
                    char text_cmd[512];
                    sprintf(text_cmd, "text=%s; mag=%.1f; x=%.0f; y=%.0f; layer=%s;", 
                           text_content, mag, final_x, final_y, layerName);
                    
                    // 调用TextMacro处理
                    TextMacro(text_cmd);
                    
                    char debug_msg[256];
                    sprintf(debug_msg, "Text array element (%d,%d): text='%s', base=(%.0f,%.0f), text_offset=(%.0f,%.0f), array_offset=(%.0f,%.0f), final=(%.0f,%.0f)", 
                           ix+1, iy+1, text_content, base_x, base_y, text_offset_x, text_offset_y, ox, oy, final_x, final_y);
                    log_main(debug_msg);
                }

                sprintf(logbuf, "  -> element (%d,%d) offset=(%.1f,%.1f)", ix+1, iy+1, ox, oy);
                log_main(logbuf);
            }
        }

        sprintf(logbuf, "ArrayMacro finished: total = %d × %d = %d objects", nx, ny, nx*ny);
        log_main(logbuf);
        log_main("========== [ArrayMacro] END ==========");

    } catch (...) {
        log_main("EXCEPTION in ArrayMacro");
    }
}



// Note: Old process-based communication replaced by thread-based message queues

// =========================== 功能队列管理系统实施 ===========================

// 注册新的宏功能到队列中
int register_macro_function(const char* name, void (*function_ptr)(void), const char* description) {
    if (g_macroQueueSize >= MAX_MACRO_FUNCTIONS) {
        log_main("ERROR: Macro function queue is full");
        return -1;
    }
    
    // 检查是否已经存在同名功能
    for (int i = 0; i < g_macroQueueSize; i++) {
        if (strcmp(g_macroQueue[i].name, name) == 0) {
            log_main("WARNING: Macro function with same name already exists, updating...");
            g_macroQueue[i].function_ptr = function_ptr;
            strncpy(g_macroQueue[i].description, description, sizeof(g_macroQueue[i].description) - 1);
            g_macroQueue[i].description[sizeof(g_macroQueue[i].description) - 1] = '\0';
            g_macroQueue[i].enabled = 1;
            return i;
        }
    }
    
    // 添加新功能
    strncpy(g_macroQueue[g_macroQueueSize].name, name, sizeof(g_macroQueue[g_macroQueueSize].name) - 1);
    g_macroQueue[g_macroQueueSize].name[sizeof(g_macroQueue[g_macroQueueSize].name) - 1] = '\0';
    g_macroQueue[g_macroQueueSize].function_ptr = function_ptr;
    strncpy(g_macroQueue[g_macroQueueSize].description, description, sizeof(g_macroQueue[g_macroQueueSize].description) - 1);
    g_macroQueue[g_macroQueueSize].description[sizeof(g_macroQueue[g_macroQueueSize].description) - 1] = '\0';
    g_macroQueue[g_macroQueueSize].enabled = 1;
    
    char logMsg[200];
    sprintf(logMsg, "Registered macro function: %s (total: %d)", name, g_macroQueueSize + 1);
    log_main(logMsg);
    
    return g_macroQueueSize++;
}

// 通过名称执行宏功能
int execute_macro_by_name(const char* name) {
    for (int i = 0; i < g_macroQueueSize; i++) {
        if (strcmp(g_macroQueue[i].name, name) == 0 && g_macroQueue[i].enabled) {
            char logMsg[200];
            sprintf(logMsg, "Executing macro function: %s", name);
            log_main(logMsg);
            
            if (g_macroQueue[i].function_ptr) {
                g_macroQueue[i].function_ptr();
                return 0;  // 成功执行
            } else {
                log_main("ERROR: Function pointer is NULL");
                return -2;
            }
        }
    }
    
    char errorMsg[200];
    sprintf(errorMsg, "ERROR: Macro function not found or disabled: %s", name);
    log_main(errorMsg);
    return -1; // 未找到功能
}

// 列出所有可用的宏功能
void list_available_macros(char* result_buffer, int buffer_size) {
    if (!result_buffer || buffer_size <= 0) return;
    
    strcpy(result_buffer, "Available macro functions:\n");
    
    for (int i = 0; i < g_macroQueueSize; i++) {
        char entry[300];
        sprintf(entry, "  %d. %s - %s [%s]\n", 
                i + 1, 
                g_macroQueue[i].name, 
                g_macroQueue[i].description, 
                g_macroQueue[i].enabled ? "enabled" : "disabled");
        
        if (strlen(result_buffer) + strlen(entry) < buffer_size - 1) {
            strcat(result_buffer, entry);
        } else {
            strcat(result_buffer, "  ... (truncated)\n");
            break;
        }
    }
    
    if (g_macroQueueSize == 0) {
        strcat(result_buffer, "  No macro functions registered.\n");
    }
}

// 解析命令和参数 - 带调试日志
void parse_command_with_params(const char* command, char* cmd_name, char* params) {
    log_main("DEBUG: parse_command_with_params called");
    
    if (!command) {
        log_main("ERROR: command pointer is NULL");
        return;
    }
    
    if (!cmd_name || !params) {
        log_main("ERROR: output buffer pointers are NULL");
        return;
    }
    
    char debug_msg[300];
    sprintf(debug_msg, "DEBUG: parsing command: [%s]", command);
    log_main(debug_msg);
    
    // 初始化输出缓冲区
    cmd_name[0] = '\0';
    params[0] = '\0';
    
    // 查找第一个空格分隔符
    const char* space_pos = strchr(command, ' ');
    
    if (space_pos) {
        // 提取命令名称
        int cmd_len = space_pos - command;
        if (cmd_len > 63) cmd_len = 63; // 限制长度
        strncpy(cmd_name, command, cmd_len);
        cmd_name[cmd_len] = '\0';
        
        // 提取参数（跳过空格）
        strncpy(params, space_pos + 1, 255);
        params[255] = '\0';
        
        sprintf(debug_msg, "DEBUG: command = [%s], params = [%s]", cmd_name, params);
        log_main(debug_msg);
    } else {
        // 没有参数，整条命令就是命令名称
        strncpy(cmd_name, command, 63);
        cmd_name[63] = '\0';
        
        sprintf(debug_msg, "DEBUG: command = [%s], no params", cmd_name);
        log_main(debug_msg);
    }
    
    log_main("DEBUG: parse_command_with_params completed");
}

// 增强的晶体管宏功能，支持参数化 - 独立实现
void enhanced_transistor_macro(const char* params) {
    log_main("=== Enhanced TransistorMacro START (Independent Implementation) ===");
    
    // 清空之前的结果
    memset(&g_lastResult, 0, sizeof(g_lastResult));
    g_lastResult.success = 0;
    strcpy(g_lastResult.error_message, "");
    strcpy(g_lastResult.custom_response, "");
    
    // 解析参数以自定义晶体管尺寸和图层
    float W = 45000.0f;  // 默认宽度
    float L = 2000.0f;   // 默认长度
    float X = 0.0f;      // 默认X位置
    float Y = 0.0f;      // 默认Y位置
    
    // 图层参数，带默认值
    char gateLayerName[64] = "Gate";
    char channelLayerName[64] = "mos2";
    char sourceLayerName[64] = "Source";
    
    if (params && strlen(params) > 0) {
        log_main("DEBUG: Parsing parameters for enhanced transistor");
        
        // 创建参数副本用于解析
        char params_copy[256];
        strncpy(params_copy, params, sizeof(params_copy) - 1);
        params_copy[sizeof(params_copy) - 1] = '\0';
        
        // 使用简单的字符串解析
        char* current_pos = params_copy;
        char* token_start;
        
        while (current_pos && *current_pos) {
            // 跳过前导空格和逗号
            while (*current_pos == ' ' || *current_pos == ',') {
                current_pos++;
            }
            
            if (*current_pos == '\0') break;
            
            token_start = current_pos;
            
            // 找到下一个逗号或字符串末尾
            while (*current_pos && *current_pos != ',') {
                current_pos++;
            }
            
            // 临时终止当前token
            char saved_char = *current_pos;
            *current_pos = '\0';
            
            // 解析token
            if (strncmp(token_start, "W=", 2) == 0) {
                W = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "L=", 2) == 0) {
                L = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "X=", 2) == 0) {
                X = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "Y=", 2) == 0) {
                Y = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "gatelayer=", 10) == 0) {
                strncpy(gateLayerName, token_start + 10, sizeof(gateLayerName) - 1);
                gateLayerName[sizeof(gateLayerName) - 1] = '\0';
            } else if (strncmp(token_start, "channellayer=", 13) == 0) {
                strncpy(channelLayerName, token_start + 13, sizeof(channelLayerName) - 1);
                channelLayerName[sizeof(channelLayerName) - 1] = '\0';
            } else if (strncmp(token_start, "sourcelayer=", 12) == 0) {
                strncpy(sourceLayerName, token_start + 12, sizeof(sourceLayerName) - 1);
                sourceLayerName[sizeof(sourceLayerName) - 1] = '\0';
            }
            
            // 恢复字符并移动到下一个位置
            *current_pos = saved_char;
            if (*current_pos == ',') {
                current_pos++;
            }
        }
    }
    
    char paramMsg[300];
    sprintf(paramMsg, "Enhanced transistor parameters: W=%.0f, L=%.0f, X=%.0f, Y=%.0f, Gate=%s, Channel=%s, Source=%s", 
            W, L, X, Y, gateLayerName, channelLayerName, sourceLayerName);
    log_main(paramMsg);
    
    // === 以下是复制自TransistorMacro的核心逻辑 ===
    
    LPoint gatePolygon[4], mos2Polygon[4], source1Polygon[4], source2Polygon[4];
    float mos2_tolerance, contact_tolerance_w, contact_tolerance_l, gate_tolerance_w, gate_tolerance_l;
    
    try {
        log_main("Attempting to get visible cell for enhanced transistor...");
        LCell Cell_Draw = LCell_GetVisible();
        
        if (Cell_Draw == NULL) {
            log_main("ERROR: LCell_GetVisible() returned NULL");
            log_main("Trying alternative method to get current cell...");
            
            LFile currentFile = LFile_GetVisible();
            if (currentFile != NULL) {
                log_main("Found visible file, trying to get its cells...");
                Cell_Draw = LCell_GetList(currentFile);
                if (Cell_Draw != NULL) {
                    log_main("Successfully got first cell from visible file using LCell_GetList");
                } else {
                    log_main("ERROR: No cells found in visible file");
                    strcpy(g_lastResult.error_message, "No cells found in visible file");
                    return;
                }
            } else {
                log_main("ERROR: No visible file found either");
                strcpy(g_lastResult.error_message, "No visible file found");
                return;
            }
        } else {
            log_main("Successfully got visible cell using LCell_GetVisible()");
        }
        
        // 验证Cell_Draw是否有效
        if (Cell_Draw == NULL) {
            log_main("CRITICAL ERROR: Cell_Draw is still NULL after all attempts");
            strcpy(g_lastResult.error_message, "Could not obtain valid cell");
            return;
        }
        
        log_main("DEBUG: Cell_Draw pointer is valid, proceeding...");
        
        LFile File_Draw = LCell_GetFile(Cell_Draw);
        if (File_Draw == NULL) {
            log_main("ERROR: Could not get file from cell");
            strcpy(g_lastResult.error_message, "Could not get file from cell");
            return;
        } else {
            log_main("DEBUG: Successfully got file from cell");
        }
        
        // 提取单元名称
        char cellNameBuffer[128];
        if (LCell_GetName(Cell_Draw, cellNameBuffer, sizeof(cellNameBuffer)) != NULL) {
            strncpy(g_lastResult.layout_data.cell_name, cellNameBuffer, sizeof(g_lastResult.layout_data.cell_name) - 1);
            g_lastResult.layout_data.cell_name[sizeof(g_lastResult.layout_data.cell_name) - 1] = '\0';
        } else {
            strcpy(g_lastResult.layout_data.cell_name, "Unknown");
        }
        
        // 检查必需的层 - 使用解析到的图层名称
        LLayer gateLayer = LLayer_Find(File_Draw, gateLayerName);
        LLayer mos2Layer = LLayer_Find(File_Draw, channelLayerName);
        LLayer sourceLayer = LLayer_Find(File_Draw, sourceLayerName);
        
        // 构建层信息字符串
        char layerInfo[512] = "Layers: ";
        char tempMsg[100];
        
        if (gateLayer == NULL) {
            sprintf(tempMsg, "%s(missing) ", gateLayerName);
            strcat(layerInfo, tempMsg);
        } else {
            sprintf(tempMsg, "%s(ok) ", gateLayerName);
            strcat(layerInfo, tempMsg);
        }
        if (mos2Layer == NULL) {
            sprintf(tempMsg, "%s(missing) ", channelLayerName);
            strcat(layerInfo, tempMsg);
        } else {
            sprintf(tempMsg, "%s(ok) ", channelLayerName);
            strcat(layerInfo, tempMsg);
        }
        if (sourceLayer == NULL) {
            sprintf(tempMsg, "%s(missing) ", sourceLayerName);
            strcat(layerInfo, tempMsg);
        } else {
            sprintf(tempMsg, "%s(ok) ", sourceLayerName);
            strcat(layerInfo, tempMsg);
        }
        strcpy(g_lastResult.layout_data.layer_info, layerInfo);
        
        if (gateLayer == NULL || mos2Layer == NULL || sourceLayer == NULL) {
            sprintf(tempMsg, "Required layers (%s, %s, %s) not found in layer map", 
                    gateLayerName, channelLayerName, sourceLayerName);
            strcpy(g_lastResult.error_message, tempMsg);
            return;
        }
        
        // 使用解析出的参数设置位置和尺寸
        LPoint Translation = LPoint_Set(X, Y);
        
        // 记录几何参数
        g_lastResult.layout_data.width = W;
        g_lastResult.layout_data.length = L;
        g_lastResult.layout_data.x_position = X;
        g_lastResult.layout_data.y_position = Y;
        
        sprintf(paramMsg, "Creating enhanced transistor with W=%.0f, L=%.0f at position (%.0f, %.0f)", W, L, X, Y);
        log_main(paramMsg);
        
        // 计算几何形状（使用解析出的W和L参数）
        mos2_tolerance = 2000;
        mos2Polygon[0] = LPoint_Set(-W / 2 + Translation.x, -(L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[1] = LPoint_Set(W / 2 + Translation.x, -(L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[2] = LPoint_Set(W / 2 + Translation.x, (L / 2 + mos2_tolerance) + Translation.y);
        mos2Polygon[3] = LPoint_Set(-W / 2 + Translation.x, (L / 2 + mos2_tolerance) + Translation.y);
        
        gate_tolerance_w = 7500;
        gate_tolerance_l = 4000;
        gatePolygon[0] = LPoint_Set(-(W / 2+gate_tolerance_w) + Translation.x, -(L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[1] = LPoint_Set((W / 2 + gate_tolerance_w) + Translation.x, -(L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[2] = LPoint_Set((W / 2 + gate_tolerance_w) + Translation.x, (L / 2 + gate_tolerance_l) + Translation.y);
        gatePolygon[3] = LPoint_Set(-(W / 2 + gate_tolerance_w) + Translation.x, (L / 2 + gate_tolerance_l) + Translation.y);
        
        contact_tolerance_w = 5000;
        contact_tolerance_l = 10000;
        source1Polygon[0] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) +(L/2+contact_tolerance_l/2) +Translation.y);
        source1Polygon[1] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source1Polygon[2] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source1Polygon[3] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) + (L / 2 + contact_tolerance_l / 2) + Translation.y);
        
        source2Polygon[0] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) -(L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[1] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, -(contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[2] = LPoint_Set((W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        source2Polygon[3] = LPoint_Set(-(W / 2 + contact_tolerance_w) + +Translation.x, (contact_tolerance_l / 2) - (L / 2 + contact_tolerance_l / 2) + Translation.y);
        
        log_main("Creating enhanced transistor polygon objects...");
        
        // 在创建多边形前进行最终验证
        if (Cell_Draw == NULL) {
            log_main("FATAL ERROR: Cell_Draw is NULL before polygon creation");
            strcpy(g_lastResult.error_message, "Cell_Draw is NULL");
            return;
        }
        
        if (gateLayer == NULL) {
            log_main("FATAL ERROR: gateLayer is NULL before polygon creation");
            strcpy(g_lastResult.error_message, "gateLayer is NULL");
            return;
        }
        
        if (mos2Layer == NULL) {
            log_main("FATAL ERROR: mos2Layer is NULL before polygon creation");
            strcpy(g_lastResult.error_message, "mos2Layer is NULL");
            return;
        }
        
        if (sourceLayer == NULL) {
            log_main("FATAL ERROR: sourceLayer is NULL before polygon creation");
            strcpy(g_lastResult.error_message, "sourceLayer is NULL");
            return;
        }
        
        log_main("DEBUG: All pointers validated, creating polygons...");
        
        // 调试：检查文件和单元状态
        char status_debug[200];
        sprintf(status_debug, "DEBUG: Cell_Draw pointer = %p, File_Draw pointer = %p", (void*)Cell_Draw, (void*)File_Draw);
        log_main(status_debug);
        
        sprintf(status_debug, "DEBUG: gateLayer pointer = %p, mos2Layer pointer = %p, sourceLayer pointer = %p", 
                (void*)gateLayer, (void*)mos2Layer, (void*)sourceLayer);
        log_main(status_debug);
        
        // 调试：打印几何坐标
        char geom_debug[300];
        sprintf(geom_debug, "DEBUG: Gate polygon: (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f) (%.0f,%.0f)", 
                (float)gatePolygon[0].x, (float)gatePolygon[0].y,
                (float)gatePolygon[1].x, (float)gatePolygon[1].y,
                (float)gatePolygon[2].x, (float)gatePolygon[2].y,
                (float)gatePolygon[3].x, (float)gatePolygon[3].y);
        log_main(geom_debug);
        
        // 尝试单独创建每个多边形，并捕获异常
        log_main("DEBUG: Creating gate polygon...");
        try {
            LPolygon_New(Cell_Draw, gateLayer, gatePolygon, 4);
            log_main("DEBUG: Gate polygon created successfully");
        } catch (...) {
            log_main("FATAL ERROR: Exception during gate polygon creation");
            strcpy(g_lastResult.error_message, "Failed to create gate polygon");
            return;
        }
        
        log_main("DEBUG: Creating mos2 polygon...");
        try {
            LPolygon_New(Cell_Draw, mos2Layer, mos2Polygon, 4);
            log_main("DEBUG: MOS2 polygon created successfully");
        } catch (...) {
            log_main("FATAL ERROR: Exception during mos2 polygon creation");
            strcpy(g_lastResult.error_message, "Failed to create mos2 polygon");
            return;
        }
        
        log_main("DEBUG: Creating source1 polygon...");
        try {
            LPolygon_New(Cell_Draw, sourceLayer, source1Polygon, 4);
            log_main("DEBUG: Source1 polygon created successfully");
        } catch (...) {
            log_main("FATAL ERROR: Exception during source1 polygon creation");
            strcpy(g_lastResult.error_message, "Failed to create source1 polygon");
            return;
        }
        
        log_main("DEBUG: Creating source2 polygon...");
        try {
            LPolygon_New(Cell_Draw, sourceLayer, source2Polygon, 4);
            log_main("DEBUG: Source2 polygon created successfully");
        } catch (...) {
            log_main("FATAL ERROR: Exception during source2 polygon creation");
            strcpy(g_lastResult.error_message, "Failed to create source2 polygon");
            return;
        }
        
        // 记录多边形数量
        g_lastResult.layout_data.polygon_count = 4;
        
        log_main("Enhanced transistor polygons created successfully");
        
        // 添加时间戳
        SYSTEMTIME st;
        GetSystemTime(&st);
        sprintf(g_lastResult.layout_data.timestamp, "%04d-%02d-%02d %02d:%02d:%02d", 
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        
        // 更新显示
        LCell_MakeVisible(Cell_Draw);
        LCell_HomeView(Cell_Draw);
        
        // 创建详细的响应信息
        sprintf(g_lastResult.custom_response, 
                "Enhanced Transistor created successfully! Cell: %s, W=%.0f, L=%.0f, Position=(%.0f,%.0f), Polygons: %d, Time: %s",
                g_lastResult.layout_data.cell_name,
                g_lastResult.layout_data.width,
                g_lastResult.layout_data.length,
                g_lastResult.layout_data.x_position,
                g_lastResult.layout_data.y_position,
                g_lastResult.layout_data.polygon_count,
                g_lastResult.layout_data.timestamp);
        
        g_lastResult.success = 1;  // 标记成功
        log_main("Enhanced TransistorMacro completed successfully");
        
    } catch (...) {
        log_main("EXCEPTION: Error occurred in enhanced_transistor_macro");
        g_lastResult.success = 0;
        strcpy(g_lastResult.error_message, "Exception occurred during enhanced transistor creation");
    }
    
    log_main("=== Enhanced TransistorMacro END ===");
}

// 忆阻器宏功能，支持参数化配置
void memristor_macro(const char* params) {
    log_main("=== MemristorMacro START ===");
    
    // 清空之前的结果
    memset(&g_lastResult, 0, sizeof(g_lastResult));
    g_lastResult.success = 0;
    strcpy(g_lastResult.error_message, "");
    strcpy(g_lastResult.custom_response, "");
    
    // 解析参数
    float lineWidth = 5000.0f;      // 默认线宽5um
    float extension = 2000.0f;      // 默认突出长度2um
    float X = 170000.0f;            // 默认X坐标
    float Y = 310000.0f;            // 默认Y坐标
    int rotated = 0;                // 默认不旋转（0=不旋转，1=旋转）
    
    // 图层参数，带默认值
    char topLayerName[64] = "Topelectrode";
    char bottomLayerName[64] = "contact";
    
    if (params && strlen(params) > 0) {
        log_main("DEBUG: Parsing parameters for memristor");
        
        // 创建参数副本用于解析
        char params_copy[512];
        strncpy(params_copy, params, sizeof(params_copy) - 1);
        params_copy[sizeof(params_copy) - 1] = '\0';
        
        // 使用简单的字符串解析
        char* current_pos = params_copy;
        char* token_start;
        
        while (current_pos && *current_pos) {
            // 跳过前导空格和逗号
            while (*current_pos == ' ' || *current_pos == ',') {
                current_pos++;
            }
            
            if (*current_pos == '\0') break;
            
            token_start = current_pos;
            
            // 找到下一个逗号或字符串末尾
            while (*current_pos && *current_pos != ',') {
                current_pos++;
            }
            
            // 临时终止当前token
            char saved_char = *current_pos;
            *current_pos = '\0';
            
            // 解析token
            if (strncmp(token_start, "linewidth=", 10) == 0) {
                lineWidth = (float)atof(token_start + 10);
            } else if (strncmp(token_start, "extension=", 10) == 0) {
                extension = (float)atof(token_start + 10);
            } else if (strncmp(token_start, "X=", 2) == 0) {
                X = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "Y=", 2) == 0) {
                Y = (float)atof(token_start + 2);
            } else if (strncmp(token_start, "rotated=", 8) == 0) {
                rotated = atoi(token_start + 8);
            } else if (strncmp(token_start, "toplayer=", 9) == 0) {
                strncpy(topLayerName, token_start + 9, sizeof(topLayerName) - 1);
                topLayerName[sizeof(topLayerName) - 1] = '\0';
            } else if (strncmp(token_start, "bottomlayer=", 12) == 0) {
                strncpy(bottomLayerName, token_start + 12, sizeof(bottomLayerName) - 1);
                bottomLayerName[sizeof(bottomLayerName) - 1] = '\0';
            }
            
            // 恢复字符并移动到下一个位置
            *current_pos = saved_char;
            if (*current_pos == ',') {
                current_pos++;
            }
        }
    }
    
    char paramMsg[400];
    sprintf(paramMsg, "Memristor parameters: linewidth=%.0f, extension=%.0f, X=%.0f, Y=%.0f, rotated=%d, toplayer=%s, bottomlayer=%s", 
            lineWidth, extension, X, Y, rotated, topLayerName, bottomLayerName);
    log_main(paramMsg);
    
    // 计算忆阻器尺寸
    float totalLength = lineWidth + 2 * extension;  // 总长度 = 线宽 + 2*突出长度
    
    // 根据旋转参数决定底电极和顶电极的尺寸
    float bottomWidth, bottomHeight, topWidth, topHeight;
    
    if (rotated == 0) {
        // 默认方向：底电极为横向长方形，顶电极为纵向长方形
        bottomWidth = totalLength;   // 底电极宽度
        bottomHeight = lineWidth;    // 底电极高度  
        topWidth = lineWidth;        // 顶电极宽度
        topHeight = totalLength;     // 顶电极高度
    } else {
        // 旋转方向：底电极为纵向长方形，顶电极为横向长方形
        bottomWidth = lineWidth;     // 底电极宽度
        bottomHeight = totalLength;  // 底电极高度
        topWidth = totalLength;      // 顶电极宽度  
        topHeight = lineWidth;       // 顶电极高度
    }
    
    sprintf(paramMsg, "Calculated dimensions - Bottom: %.0fx%.0f, Top: %.0fx%.0f", 
            bottomWidth, bottomHeight, topWidth, topHeight);
    log_main(paramMsg);
    
    try {
        log_main("Attempting to get visible cell for memristor...");
        LCell Cell_Draw = LCell_GetVisible();
        
        if (Cell_Draw == NULL) {
            log_main("ERROR: LCell_GetVisible() returned NULL");
            log_main("Trying alternative method to get current cell...");
            
            LFile currentFile = LFile_GetVisible();
            if (currentFile != NULL) {
                log_main("Found visible file, trying to get its cells...");
                Cell_Draw = LCell_GetList(currentFile);
                if (Cell_Draw != NULL) {
                    log_main("Successfully got first cell from visible file using LCell_GetList");
                } else {
                    log_main("ERROR: No cells found in visible file");
                    strcpy(g_lastResult.error_message, "No cells found in visible file");
                    return;
                }
            } else {
                log_main("ERROR: No visible file found either");
                strcpy(g_lastResult.error_message, "No visible file found");
                return;
            }
        } else {
            log_main("Successfully got visible cell using LCell_GetVisible()");
        }
        
        // 验证Cell_Draw是否有效
        if (Cell_Draw == NULL) {
            log_main("CRITICAL ERROR: Cell_Draw is still NULL after all attempts");
            strcpy(g_lastResult.error_message, "Could not obtain valid cell");
            return;
        }
        
        log_main("DEBUG: Cell_Draw pointer is valid, proceeding...");
        
        LFile File_Draw = LCell_GetFile(Cell_Draw);
        if (File_Draw == NULL) {
            log_main("ERROR: Could not get file from cell");
            strcpy(g_lastResult.error_message, "Could not get file from cell");
            return;
        } else {
            log_main("DEBUG: Successfully got file from cell");
        }
        
        // 提取单元名称
        char cellNameBuffer[128];
        if (LCell_GetName(Cell_Draw, cellNameBuffer, sizeof(cellNameBuffer)) != NULL) {
            strncpy(g_lastResult.layout_data.cell_name, cellNameBuffer, sizeof(g_lastResult.layout_data.cell_name) - 1);
            g_lastResult.layout_data.cell_name[sizeof(g_lastResult.layout_data.cell_name) - 1] = '\0';
        } else {
            strcpy(g_lastResult.layout_data.cell_name, "Unknown");
        }
        
        // 检查必需的层
        LLayer topLayer = LLayer_Find(File_Draw, topLayerName);
        LLayer bottomLayer = LLayer_Find(File_Draw, bottomLayerName);
        
        // 构建层信息字符串
        char layerInfo[512] = "Layers: ";
        char tempMsg[100];
        
        if (topLayer == NULL) {
            sprintf(tempMsg, "%s(missing) ", topLayerName);
            strcat(layerInfo, tempMsg);
        } else {
            sprintf(tempMsg, "%s(ok) ", topLayerName);
            strcat(layerInfo, tempMsg);
        }
        if (bottomLayer == NULL) {
            sprintf(tempMsg, "%s(missing) ", bottomLayerName);
            strcat(layerInfo, tempMsg);
        } else {
            sprintf(tempMsg, "%s(ok) ", bottomLayerName);
            strcat(layerInfo, tempMsg);
        }
        strcpy(g_lastResult.layout_data.layer_info, layerInfo);
        
        if (topLayer == NULL || bottomLayer == NULL) {
            sprintf(tempMsg, "Required layers (%s, %s) not found in layer map", 
                    topLayerName, bottomLayerName);
            strcpy(g_lastResult.error_message, tempMsg);
            return;
        }
        
        // 设置位置
        LPoint Translation = LPoint_Set(X, Y);
        
        // 记录几何参数
        g_lastResult.layout_data.width = lineWidth;
        g_lastResult.layout_data.length = totalLength;
        g_lastResult.layout_data.x_position = X;
        g_lastResult.layout_data.y_position = Y;
        
        sprintf(paramMsg, "Creating memristor with linewidth=%.0f, extension=%.0f at position (%.0f, %.0f)", 
                lineWidth, extension, X, Y);
        log_main(paramMsg);
        
        // 创建底电极多边形（加上中心坐标偏移）
        LPoint bottomPolygon[4];
        float halfBottomWidth = bottomWidth / 2.0f;
        float halfBottomHeight = bottomHeight / 2.0f;
        
        bottomPolygon[0] = LPoint_Set(-halfBottomWidth + Translation.x, -halfBottomHeight + Translation.y);  // 左下
        bottomPolygon[1] = LPoint_Set( halfBottomWidth + Translation.x, -halfBottomHeight + Translation.y);  // 右下
        bottomPolygon[2] = LPoint_Set( halfBottomWidth + Translation.x,  halfBottomHeight + Translation.y);  // 右上
        bottomPolygon[3] = LPoint_Set(-halfBottomWidth + Translation.x,  halfBottomHeight + Translation.y);  // 左上
        
        // 创建顶电极多边形（加上中心坐标偏移）
        LPoint topPolygon[4];
        float halfTopWidth = topWidth / 2.0f;
        float halfTopHeight = topHeight / 2.0f;
        
        topPolygon[0] = LPoint_Set(-halfTopWidth + Translation.x, -halfTopHeight + Translation.y);  // 左下
        topPolygon[1] = LPoint_Set( halfTopWidth + Translation.x, -halfTopHeight + Translation.y);  // 右下
        topPolygon[2] = LPoint_Set( halfTopWidth + Translation.x,  halfTopHeight + Translation.y);  // 右上
        topPolygon[3] = LPoint_Set(-halfTopWidth + Translation.x,  halfTopHeight + Translation.y);  // 左上
        
        log_main("Creating bottom electrode polygon...");
        try {
            LPolygon_New(Cell_Draw, bottomLayer, bottomPolygon, 4);
            log_main("Bottom electrode polygon created successfully");
        } catch (...) {
            log_main("ERROR: Failed to create bottom electrode polygon");
            strcpy(g_lastResult.error_message, "Failed to create bottom electrode polygon");
            return;
        }
        
        log_main("Creating top electrode polygon...");
        try {
            LPolygon_New(Cell_Draw, topLayer, topPolygon, 4);
            log_main("Top electrode polygon created successfully");
        } catch (...) {
            log_main("ERROR: Failed to create top electrode polygon");
            strcpy(g_lastResult.error_message, "Failed to create top electrode polygon");
            return;
        }
        
        log_main("Memristor polygons created successfully");
        
        // 设置成功状态和响应信息
        g_lastResult.success = 1;
        g_lastResult.layout_data.polygon_count = 2;  // 2个多边形：顶电极和底电极
        
        // 生成时间戳
        SYSTEMTIME st;
        GetSystemTime(&st);
        sprintf(g_lastResult.layout_data.timestamp, "%04d-%02d-%02d %02d:%02d:%02d", 
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        
        sprintf(g_lastResult.custom_response, 
                "Memristor created successfully! Cell: %s, LineWidth=%.0f, Extension=%.0f, Position=(%.0f,%.0f), Rotated=%s, Polygons: %d, Time: %s",
                g_lastResult.layout_data.cell_name, lineWidth, extension, X, Y, 
                (rotated ? "Yes" : "No"), g_lastResult.layout_data.polygon_count, 
                g_lastResult.layout_data.timestamp);
        
        log_main("Memristor macro completed successfully");
        
    } catch (...) {
        log_main("EXCEPTION: Error occurred in memristor_macro");
        g_lastResult.success = 0;
        strcpy(g_lastResult.error_message, "Exception occurred during memristor creation");
    }
    
    log_main("=== MemristorMacro END ===");
}

// 格式化宏执行结果为响应字符串 - 带详细调试
void format_macro_result(const MacroResult* result, char* response_buffer, int buffer_size) {
    log_main("DEBUG: format_macro_result called");
    
    if (!result) {
        log_main("ERROR: result pointer is NULL");
        if (response_buffer && buffer_size > 0) {
            strcpy(response_buffer, "ERROR: Invalid result pointer");
        }
        return;
    }
    
    if (!response_buffer) {
        log_main("ERROR: response_buffer pointer is NULL");
        return;
    }
    
    if (buffer_size <= 0) {
        log_main("ERROR: buffer_size is invalid");
        return;
    }
    
    char debug_msg[200];
    sprintf(debug_msg, "DEBUG: result->success = %d", result->success);
    log_main(debug_msg);
    
    // 清空响应缓冲区
    memset(response_buffer, 0, buffer_size);
    
    if (result->success) {
        log_main("DEBUG: Formatting successful result");
        
        // 成功情况：返回详细的版图信息
        try {
            int chars_written = snprintf(response_buffer, buffer_size,
                     "SUCCESS: %s\n"
                     "Layout Info:\n"
                     "  Cell: %s\n"
                     "  Dimensions: W=%.0f, L=%.0f\n"
                     "  Position: (%.0f, %.0f)\n"
                     "  Polygons: %d\n"
                     "  Layers: %s\n"
                     "  Timestamp: %s",
                     result->custom_response,
                     result->layout_data.cell_name,
                     result->layout_data.width,
                     result->layout_data.length,
                     result->layout_data.x_position,
                     result->layout_data.y_position,
                     result->layout_data.polygon_count,
                     result->layout_data.layer_info,
                     result->layout_data.timestamp);
                     
            sprintf(debug_msg, "DEBUG: snprintf wrote %d characters", chars_written);
            log_main(debug_msg);
            
            if (chars_written >= buffer_size) {
                log_main("WARNING: Response was truncated due to buffer size");
            }
            
        } catch (...) {
            log_main("ERROR: Exception in snprintf for successful result");
            strcpy(response_buffer, "ERROR: Exception occurred while formatting successful result");
        }
        
    } else {
        log_main("DEBUG: Formatting error result");
        
        // 失败情况：返回错误信息
        try {
            const char* error_msg = (strlen(result->error_message) > 0) ? 
                                  result->error_message : "Unknown error occurred";
            
            int chars_written = snprintf(response_buffer, buffer_size, "ERROR: %s", error_msg);
            
            sprintf(debug_msg, "DEBUG: Error snprintf wrote %d characters", chars_written);
            log_main(debug_msg);
            
        } catch (...) {
            log_main("ERROR: Exception in snprintf for error result");
            strcpy(response_buffer, "ERROR: Exception occurred while formatting error result");
        }
    }
    
    sprintf(debug_msg, "DEBUG: Final response length = %d", (int)strlen(response_buffer));
    log_main(debug_msg);
    log_main("DEBUG: format_macro_result completed");
}

// 包装函数，用于兼容函数指针类型
void enhanced_transistor_macro_wrapper(void) {
    log_main("DEBUG: enhanced_transistor_macro_wrapper called - using default parameters");
    enhanced_transistor_macro("");  // 调用时传入空参数，使用默认值
}

// 新的wrapper函数，演示如何使用自定义图层参数
void enhanced_transistor_with_custom_layers_wrapper(void) {
    log_main("DEBUG: enhanced_transistor_with_custom_layers_wrapper called - using custom layer parameters");
    // 使用自定义的图层名称和参数
    enhanced_transistor_macro("W=30000, L=1500, X=5000, Y=5000, gatelayer=MyGate, channellayer=MyChannel, sourcelayer=MySource");
}

void transistor_with_custom_layers_wrapper(void) {
    log_main("DEBUG: transistor_with_custom_layers_wrapper called - using custom layer names");
    // 使用新的TransistorMacroWithLayers函数，传入自定义图层名
    TransistorMacroWithLayers("MyGate", "MyChannel", "MySource");
}

// 忆阻器包装函数，用于兼容函数指针类型
void memristor_macro_wrapper(void) {
    log_main("DEBUG: memristor_macro_wrapper called - using default parameters");
    memristor_macro("");  // 调用时传入空参数，使用默认值
}

// 忆阻器自定义参数包装函数
void memristor_with_custom_params_wrapper(void) {
    log_main("DEBUG: memristor_with_custom_params_wrapper called - using custom parameters");
    // 使用自定义参数：线宽3um，突出1.5um，位置(100000,200000)，旋转，自定义图层
    memristor_macro("linewidth=3000, extension=1500, X=100000, Y=200000, rotated=1, toplayer=MyTop, bottomlayer=MyBottom");
}

void PolygonMacro_wrapper(void){
    log_main("DEBUG: PolygoMacro_wrapper called - creat default polygon");
    PolygonMacro("0 0; 1000 0; 0 1000; 1000 1000");
}

void PathMacro_wrapper(void){
    log_main("DEBUG: PathMacro_wrapper called - creat default path");
    PathMacro("0 0; 1000 0");
}

void TextMacro_wrapper(void){
    log_main("DEBUG: TextMacro_wrapper called - create default text");
    TextMacro("text=HELLO; mag=30; x=0; y=0");
}

void CircleMacro_wrapper(void){
    log_main("DEBUG: CircleMacro_wrapper called - creat default circle");
    CircleMacro("");
}

void PieWedgeMacro_wrapper(void){
    log_main("DEBUG: PieWedgeMacro_wrapper called - creat default pie wedge");
    PieWedgeMacro("");
}

void RingWedgeMacro_wrapper(void){
    log_main("DEBUG: RingWedgeMacro_wrapper called - creat default pie wedge");
    RingWedgeMacro("");
}

void BoolExprMacro_wrapper(void){
    log_main("DEBUG: BoolexprMacro_wrapper called - operate default bool operation");
    BoolExprMacro("layer = Source; polygon -1000 -1000; 1000 -1000; -1000 1000; 1000 1000 - circle 0 0; radius = 1000");
}

void ArrayMacro_wrapper(void){
    log_main("DEBUG: ArrayMacro_wrapper called - creat default array");
    ArrayMacro("layer=Source; polygon 0 0, 1000 0, 0 1000; nx=2, dx=1000; ny=2, dy=1000");
}

// 初始化功能队列系统
void initialize_macro_queue_system(void) {
    log_main("Initializing macro queue system...");
    g_macroQueueSize = 0;
    
    // 注册默认的宏功能
    register_macro_function("transistor", TransistorMacro, "Create a standard transistor with default layers (Gate, mos2, Source)");
    register_macro_function("enhanced_transistor", enhanced_transistor_macro_wrapper, "Create a parameterized transistor with default layers");
    register_macro_function("transistor_custom_layers", transistor_with_custom_layers_wrapper, "Create a transistor with custom layer names (MyGate, MyChannel, MySource)");
    register_macro_function("enhanced_transistor_custom", enhanced_transistor_with_custom_layers_wrapper, "Create a parameterized transistor with custom layers and parameters");
    register_macro_function("memristor", memristor_macro_wrapper, "Create a memristor with default parameters (linewidth=5um, extension=2um)");
    register_macro_function("memristor_custom", memristor_with_custom_params_wrapper, "Create a memristor with custom parameters and layers");
    register_macro_function("polygon", PolygonMacro_wrapper, "creat a polygon with given points");
    register_macro_function("path", PathMacro_wrapper, "creat a path with given points");
    register_macro_function("text", TextMacro_wrapper, "creat a text with given parameters");
    register_macro_function("circle", CircleMacro_wrapper, "creat a circle with given parameters");
    register_macro_function("pie wedge", PieWedgeMacro_wrapper, "creat a pie wedge with given parameters");
    register_macro_function("ring wedge", RingWedgeMacro_wrapper, "creat a ring wedge with given parameters");
    register_macro_function("bool", BoolExprMacro_wrapper, "do bool operation with given parameters");
    register_macro_function("array", ArrayMacro_wrapper, "creat a array with given parameters");
    log_main("Macro queue system initialized with default functions");
}

// =========================== 功能队列管理系统实施结束 ===========================

// Global variables for main thread message handling
static int g_messageCount = 0;
static int g_isShutdownRequested = 0;
static volatile int g_transistorPending = 0;  // Flag to indicate transistor function should be called
static volatile int g_enhancedTransistorPending = 0;  // Flag to indicate enhanced transistor function should be called
static volatile int g_polygonPending = 0;  // Flag to indicate polygon function should be called
static volatile int g_pathPending = 0;  // Flag to indicate path function should be called
static volatile int g_textPending = 0;  // Flag to indicate text function should be called
static volatile int g_circlePending = 0;  // Flag to indicate circle function should be called
static volatile int g_piewedgePending = 0;  // Flag to indicate piewedge function should be called
static volatile int g_ringwedgePending = 0;  // Flag to indicate ringwedge function should be called
static volatile int g_boolexprPending = 0;  // Flag to indicate boolexpr function should be called
static volatile int g_arrayPending = 0;  // Flag to indicate array function should be called
static volatile int g_memristorPending = 0;  // Flag to indicate memristor function should be called
static char g_enhancedTransistorParams[256] = "";  // Parameters for enhanced transistor function
static char g_memristorParams[512] = "";  // Parameters for memristor function
static char g_polygonParams[256];  // // Parameters for polygon function
static char g_pathParams[256];  // Parameters for path function
static char g_textParams[1024]; // Parameters for text function
static char g_circleParams[256]; // Parameters for circle function
static char g_piewedgeParams[256]; // Parameters for piewedge function
static char g_ringwedgeParams[256]; // Parameters for ringwedge function
static char g_boolexprParams[1024]; // Parameters for boolexpr function
static char g_arrayParams[1024]; // Parameters for array function

// Check and process messages from socket thread (non-blocking) - Enhanced version
int check_and_process_socket_messages(void) {
    ThreadMessage msg;
    
    // Try to receive message with short timeout (non-blocking)
    if (receive_message(&g_SocketToMain, &msg, 50) != 0) {
        return 0;  // No message available
    }
    
    if (msg.type == 1) {  // Data from client
        // Log received socket message (NO printf to avoid GUI crash)
        char logMessage[1024];
        sprintf(logMessage, "Main thread received message #%d: %s", ++g_messageCount, msg.data);
        log_main(logMessage);
        
        // Parse command and parameters
        char cmd_name[64];
        char params[256];
        parse_command_with_params(msg.data, cmd_name, params);
        
        ThreadMessage response;
        response.type = 2;  // Response to client
        char responseBuffer[2048];  // 增加响应缓冲区大小以支持更长的宏功能列表
        
        // Check for special commands first
        if (strcmp(cmd_name, "shutdown") == 0) {
            log_main("Received server shutdown command from client");
            g_isShutdownRequested = 1;
            
            strcpy(responseBuffer, "[main stream] Server received: ");
            strcat(responseBuffer, msg.data);
            strcat(responseBuffer, " - Server shutting down...");
        }
        else if (strcmp(cmd_name, "list") == 0) {
            log_main("Received list command - returning available macro functions");
            
            char macroList[2048];  // 增加缓冲区大小以容纳更多宏功能
            list_available_macros(macroList, sizeof(macroList));
            sprintf(responseBuffer, "[main stream] %s", macroList);
        }
        else if (strcmp(cmd_name, "transistor") == 0) {
            log_main("Received transistor command from client - scheduling TransistorMacro for main thread execution");
            
            // Mark that transistor function should be called in main thread
            g_transistorPending = 1;
            log_main("Set g_transistorPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] TransistorMacro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "enhanced_transistor") == 0) {
            log_main("Received enhanced_transistor command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: enhanced_transistor command received, params = [%s]", params);
            log_main(debug_msg);
            
            // 与transistor命令一样，使用延迟执行机制
            g_enhancedTransistorPending = 1;
            strncpy(g_enhancedTransistorParams, params, sizeof(g_enhancedTransistorParams) - 1);
            g_enhancedTransistorParams[sizeof(g_enhancedTransistorParams) - 1] = '\0';
            log_main("Set g_enhancedTransistorPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] Enhanced TransistorMacro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "memristor") == 0) {
            log_main("Received memristor command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: memristor command received, params = [%s]", params);
            log_main(debug_msg);
            
            // 与transistor命令一样，使用延迟执行机制
            g_memristorPending = 1;
            strncpy(g_memristorParams, params, sizeof(g_memristorParams) - 1);
            g_memristorParams[sizeof(g_memristorParams) - 1] = '\0';
            log_main("Set g_memristorPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] MemristorMacro scheduled for main thread execution");
        }
         else if (strcmp(cmd_name, "polygon") == 0 ) {
            log_main("Received polygon command from client - scheduling for main thread execution");

            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: polygon command received, params = [%s]", params);
            log_main(debug_msg);

            g_polygonPending = 1;
            strncpy(g_polygonParams, params, sizeof(g_polygonParams) - 1);
            g_polygonParams[sizeof(g_polygonParams) - 1] = '\0';
            log_main("Set g_polygonPending flag for main thread execution");

            strcpy(responseBuffer, "[main stream] Polygon Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "path") == 0) {
            log_main("Received path command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: path command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_pathPending = 1;
            strncpy(g_pathParams, params, sizeof(g_pathParams) - 1);
            g_pathParams[sizeof(g_pathParams) - 1] = '\0';
            log_main("Set g_pathPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] Path Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "text") == 0) {
            log_main("Received text command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: text command received, params = [%s]", params);
            log_main(debug_msg);
            
            // 参数格式处理：
            // 如果params不包含"text="，则将整个params作为文字内容
            // 例如: "text HELLO" -> "text=HELLO"
            // 例如: "text text=HELLO; mag=40" -> 保持原样
            char processed_params[1024];
            if (strlen(params) > 0 && strstr(params, "text=") == NULL) {
                // 简单文本格式，转换为标准格式
                sprintf(processed_params, "text=%s", params);
            } else {
                // 已经是标准格式或无参数
                strncpy(processed_params, params, sizeof(processed_params) - 1);
                processed_params[sizeof(processed_params) - 1] = '\0';
            }
            
            g_textPending = 1;
            strncpy(g_textParams, processed_params, sizeof(g_textParams) - 1);
            g_textParams[sizeof(g_textParams) - 1] = '\0';
            log_main("Set g_textPending flag for main thread execution");
            
            sprintf(responseBuffer, "[main stream] Text Macro scheduled: '%s'", processed_params);
        }
        else if (strcmp(cmd_name, "circle") == 0) {
            log_main("Received circle command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: circle command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_circlePending = 1;
            strncpy(g_circleParams, params, sizeof(g_circleParams) - 1);
            g_circleParams[sizeof(g_circleParams) - 1] = '\0';
            log_main("Set g_circlePending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] Circle Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "piewedge") == 0 || strcmp(cmd_name, "pie_wedge") == 0 ) {
            log_main("Received pie_wedge command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: pie_wedge command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_piewedgePending = 1;
            strncpy(g_piewedgeParams, params, sizeof(g_piewedgeParams) - 1);
            g_piewedgeParams[sizeof(g_piewedgeParams) - 1] = '\0';
            log_main("Set g_piewedgePending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] piewedge Macro scheduled for main thread execution");
        }
         else if (strcmp(cmd_name, "ringwedge") == 0 || strcmp(cmd_name, "ring_wedge") == 0 ) {
            log_main("Received ring_wedge command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: ring_wedge command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_ringwedgePending = 1;
            strncpy(g_ringwedgeParams, params, sizeof(g_ringwedgeParams) - 1);
            g_ringwedgeParams[sizeof(g_ringwedgeParams) - 1] = '\0';
            log_main("Set g_ringwedgePending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] ringwedge Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "boolexpr") == 0) {
            log_main("Received boolexpr command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: boolexpr command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_boolexprPending = 1;
            strncpy(g_boolexprParams, params, sizeof(g_boolexprParams) - 1);
            g_boolexprParams[sizeof(g_boolexprParams) - 1] = '\0';
            log_main("Set g_boolexprPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] boolexpr Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "array") == 0) {
            log_main("Received array command - scheduling for main thread execution");
            
            char debug_msg[300];
            sprintf(debug_msg, "DEBUG: array command received, params = [%s]", params);
            log_main(debug_msg);
            
            g_arrayPending = 1;
            strncpy(g_arrayParams, params, sizeof(g_arrayParams) - 1);
            g_arrayParams[sizeof(g_arrayParams) - 1] = '\0';
            log_main("Set g_arrayPending flag for main thread execution");
            
            strcpy(responseBuffer, "[main stream] array Macro scheduled for main thread execution");
        }
        else if (strcmp(cmd_name, "execute") == 0) {
            log_main("Received execute command - attempting to execute macro by name");
            
            if (strlen(params) > 0) {
                int result = execute_macro_by_name(params);
                if (result == 0) {
                    // 成功执行，返回格式化的结果
                    format_macro_result(&g_lastResult, responseBuffer, sizeof(responseBuffer));
                } else if (result == -1) {
                    sprintf(responseBuffer, "[main stream] ERROR: Macro function '%s' not found or disabled", params);
                } else {
                    sprintf(responseBuffer, "[main stream] ERROR: Failed to execute macro function '%s'", params);
                }
            } else {
                strcpy(responseBuffer, "[main stream] ERROR: Execute command requires function name parameter");
            }
        }
        else if (strcmp(cmd_name, "status") == 0) {
            log_main("Received status command - returning last execution result");
            
            if (g_lastResult.success) {
                format_macro_result(&g_lastResult, responseBuffer, sizeof(responseBuffer));
            } else {
                strcpy(responseBuffer, "[main stream] No successful macro execution recorded");
            }
        }
        else {
            // 尝试作为宏功能名称执行
            int result = execute_macro_by_name(cmd_name);
            if (result == 0) {
                // 成功执行，返回格式化的结果
                format_macro_result(&g_lastResult, responseBuffer, sizeof(responseBuffer));
            } else {
                // 传统的业务逻辑：添加"main stream"前缀到消息
                strcpy(responseBuffer, "[main stream] Server received: ");
                strcat(responseBuffer, msg.data);
                strcat(responseBuffer, " (Unknown command - treated as message)");
            }
        }
        
        send_response:
        // Log processed message
        sprintf(logMessage, "Main thread processed message: %s", responseBuffer);
        log_main(logMessage);
        
        // Send processed response back to socket thread for transmission
        response.length = strlen(responseBuffer);
        strcpy(response.data, responseBuffer);
        
        if (send_message(&g_MainToSocket, &response) != 0) {
            log_main("ERROR: Failed to send response to socket thread");
            return -1;
        }
    }
    else if (msg.type == 3) {  // Shutdown signal
        log_main("Socket thread requested shutdown");
        g_isShutdownRequested = 1;
    }
    
    return 1;  // Message processed
}

// Check if shutdown was requested
int is_shutdown_requested(void) {
    return g_isShutdownRequested;
}

// Check and execute pending transistor function in main thread
void check_and_execute_transistor_macro(void) {
    if (g_transistorPending) {
        log_main("Executing TransistorMacro in main thread context");
        g_transistorPending = 0;  // Reset flag
        
        try {
            TransistorMacro();  // Call in main thread - should be safe now
            log_main("TransistorMacro executed successfully in main thread");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing TransistorMacro in main thread");
        }
    }
}

// Main thread logic - COMPLETELY NON-BLOCKING for GUI safety
int main_thread_logic(void) {
    log_main("Main Thread Started (NON-BLOCKING MODE)");
    log_main("Main thread will process messages and return immediately");
    log_main("Socket thread is handling network connections on port 8888");
    log_main("Connect to localhost:8888 to send messages");
    log_main("Send 'shutdown' to stop server");
    
    // Process any immediate messages (short timeout only)
    int messageCount = 0;
    for (int i = 0; i < 10; i++) {  // Maximum 10 quick checks
        int result = check_and_process_socket_messages();
        if (result > 0) {
            messageCount++;
        } else if (result < 0) {
            log_main("Error in message processing");
            break;
        }
        
        // Very short sleep between checks
        Sleep(10);  // Only 10ms sleep
    }
    
    char completionMsg[200];
    sprintf(completionMsg, "Main thread completed initial message check (%d messages processed)", messageCount);
    log_main(completionMsg);
    log_main("Main thread returning to GUI - socket thread continues in background");
    
    return 0;  // Return immediately to GUI
}

// Socket thread function - handles all socket operations
int socket_thread_main(void) {
    log_socket("Socket Thread Started");
    
    // 1. Load DLL
    HMODULE hDll = LoadLibraryA("D:\\leditapi\\ddl_test.dll");
    if (!hDll) {
        log_socket("Thread ERROR: Unable to load ddl_test.dll");
        return -1;
    }
    
    // 2. Get DLL functions
    init_network = (InitNetworkFunc)GetProcAddress(hDll, "init_network");
    create_socket = (CreateSocketFunc)GetProcAddress(hDll, "create_socket");
    bind_socket = (BindSocketFunc)GetProcAddress(hDll, "bind_socket");
    listen_socket = (ListenSocketFunc)GetProcAddress(hDll, "listen_socket");
    accept_connection = (AcceptConnectionFunc)GetProcAddress(hDll, "accept_connection");
    receive_data = (ReceiveDataFunc)GetProcAddress(hDll, "receive_data");
    send_data = (SendDataFunc)GetProcAddress(hDll, "send_data");
    close_socket = (CloseSocketFunc)GetProcAddress(hDll, "close_socket");
    cleanup_network = (CleanupNetworkFunc)GetProcAddress(hDll, "cleanup_network");
    dll_htons = (HtonsFunc)GetProcAddress(hDll, "dll_htons");
    
    // Check critical functions
    if (!init_network || !create_socket || !receive_data) {
        log_socket("Child ERROR: Unable to get critical functions from DLL");
        FreeLibrary(hDll);
        return -1;
    }
    
    // 3. Initialize network
    if (init_network() != 0) {
        log_socket("Child ERROR: Network initialization failed");
        FreeLibrary(hDll);
        return -1;
    }
    
    // 4. Create socket
    void* serverSocket = create_socket();
    if (!serverSocket || serverSocket == (void*)(~0)) {
        log_socket("Child ERROR: Socket creation failed");
        cleanup_network();
        FreeLibrary(hDll);
        return -1;
    }
    
    // 5. Set server address and bind
    struct simple_addr serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.family = 2;        // AF_INET = 2
    serverAddr.ip = 0;           // INADDR_ANY = 0
    serverAddr.port = dll_htons(8888);
    
    if (bind_socket(serverSocket, &serverAddr) != 0) {
        log_socket("Child ERROR: Port binding failed");
        close_socket(serverSocket);
        cleanup_network();
        FreeLibrary(hDll);
        return -1;
    }
    
    // 6. Start listening
    if (listen_socket(serverSocket, 5) != 0) {
        log_socket("Child ERROR: Listen failed");
        close_socket(serverSocket);
        cleanup_network();
        FreeLibrary(hDll);
        return -1;
    }
    
    log_socket("Thread: Socket listening on port 8888");
    
    // 7. Main loop - handle client connections and communicate with main thread
    log_socket("Thread: Using message queues for communication with main thread");
    
    int clientCount = 0;
    while (!g_threadShutdown) {  // Check for shutdown signal
        // Accept client connection
        struct simple_addr clientAddr;
        int addrLen = sizeof(clientAddr);
        void* clientSocket = accept_connection(serverSocket, &clientAddr, &addrLen);
        
        if (!clientSocket || clientSocket == (void*)(~0)) {
            continue;
        }
        
        clientCount++;
        char connectLogMessage[100];
        sprintf(connectLogMessage, "Thread: Client #%d connected", clientCount);
        log_socket(connectLogMessage);
        
        // Data receiving loop
        char buffer[2048];  // 增加接收缓冲区大小以支持更长的响应
        while (!g_threadShutdown) {  // Check for shutdown signal
            // Clear buffer
            memset(buffer, 0, sizeof(buffer));
            
            // Receive data from client
            int bytesReceived = receive_data(clientSocket, buffer, sizeof(buffer) - 1);
            
            if (bytesReceived > 0) {
                buffer[bytesReceived] = '\0';
                
                // Send message to main thread via queue
                ThreadMessage msg;
                msg.type = 1;  // Data from client
                msg.length = bytesReceived;
                strcpy(msg.data, buffer);
                
                if (send_message(&g_SocketToMain, &msg) != 0) {
                    log_socket("Thread ERROR: Failed to send message to main thread");
                    break;
                } else {
                    log_socket("Thread: Successfully sent message to main thread");
                }
                
                // Wait for response from main thread
                ThreadMessage response;
                if (receive_message(&g_MainToSocket, &response, 5000) != 0) {
                    log_socket("Thread ERROR: Failed to read response from main thread (timeout)");
                    break;
                } else {
                    log_socket("Thread: Successfully received response from main thread");
                }
                
                // Send response to client
                if (response.type == 2) {  // Response to client
                    send_data(clientSocket, response.data, strlen(response.data));
                }
                
                // Special command handling
                if (strcmp(buffer, "quit") == 0) {
                    break;
                }
                else if (strcmp(buffer, "shutdown") == 0) {
                    close_socket(clientSocket);
                    goto cleanup_thread_and_exit;
                }
            }
            else if (bytesReceived == 0) {
                char disconnectLogMessage[100];
                sprintf(disconnectLogMessage, "Thread: Client #%d disconnected", clientCount);
                log_socket(disconnectLogMessage);
                break;
            }
            else {
                log_socket("Thread: Error receiving data");
                break;
            }
        }
        
        // Close client connection
        close_socket(clientSocket);
        char closeLogMessage[100];
        sprintf(closeLogMessage, "Thread: Client #%d connection closed", clientCount);
        log_socket(closeLogMessage);
    }
    
cleanup_thread_and_exit:
    // Send shutdown signal to main thread
    {
        ThreadMessage shutdownMsg;
        shutdownMsg.type = 3;  // Shutdown signal
        shutdownMsg.length = 0;
        strcpy(shutdownMsg.data, "");
        
        send_message(&g_SocketToMain, &shutdownMsg);
    }
    
    // Cleanup resources
    close_socket(serverSocket);
    cleanup_network();
    FreeLibrary(hDll);
    log_socket("Thread: Socket thread stopped");
    
    return 0;
}

// Socket thread wrapper function
unsigned __stdcall socket_thread_function(void* arg) {
    log_socket("Socket thread wrapper started");
    
    // Run the socket thread main function
    int result = socket_thread_main();
    
    char resultMsg[100];
    sprintf(resultMsg, "Socket thread finished with result: %d", result);
    log_socket(resultMsg);
    
    g_threadShutdown = 1;  // Signal shutdown
    return result;
}

// Message processing thread - runs continuously in background
unsigned __stdcall message_processing_thread_function(void* arg) {
    log_main("Message processing thread started");
    
    // Global variables for message handling
    int messageCount = 0;
    
    while (!g_threadShutdown) {
        // Process messages from socket thread (non-blocking)
        int result = check_and_process_socket_messages();
        if (result > 0) {
            messageCount++;
            char countMsg[100];
            sprintf(countMsg, "Processed message #%d in background thread", messageCount);
            log_main(countMsg);
        } else if (result < 0) {
            log_main("Error in background message processing");
            break;
        }
        
        // Note: TransistorMacro execution is now handled by main thread timer
        
        // Small sleep to prevent excessive CPU usage
        Sleep(50);  // 50ms sleep - this is OK in background thread
    }
    
    char finalMsg[100];
    sprintf(finalMsg, "Message processing thread finished. Total messages processed: %d", messageCount);
    log_main(finalMsg);
    return 0;
}

// Start socket thread with NON-BLOCKING startup
int start_socket_thread(void) {
    log_main("Starting socket thread (NON-BLOCKING startup)");
    
    // Reset shutdown flag
    g_threadShutdown = 0;
    
    // Create the socket handling thread
    unsigned threadId;
    g_hSocketThread = (HANDLE)_beginthreadex(NULL, 0, socket_thread_function, NULL, 0, &threadId);
    
    if (g_hSocketThread == NULL) {
        DWORD error = GetLastError();
        char errorMsg[200];
        sprintf(errorMsg, "ERROR: Failed to create socket thread. Error code: %lu", error);
        log_main(errorMsg);
        return -1;
    }
    
    log_main("Socket thread created successfully");
    
    // Minimal wait - just enough to confirm thread started
    Sleep(100);  // Only 100ms wait instead of 1500ms
    
    log_main("Socket thread startup initiated - returning to GUI immediately");
    return 0;
}

// Start message processing thread
int start_message_thread(void) {
    log_main("Starting message processing thread");
    
    // Create the message processing thread
    unsigned threadId;
    g_hMessageThread = (HANDLE)_beginthreadex(NULL, 0, message_processing_thread_function, NULL, 0, &threadId);
    
    if (g_hMessageThread == NULL) {
        DWORD error = GetLastError();
        char errorMsg[200];
        sprintf(errorMsg, "ERROR: Failed to create message processing thread. Error code: %lu", error);
        log_main(errorMsg);
        return -1;
    }
    
    log_main("Message processing thread created successfully");
    
    // Minimal wait
    Sleep(50);
    
    log_main("Message processing thread startup completed");
    return 0;
}

// Cleanup all thread resources - NON-BLOCKING
void cleanup_threads(void) {
    log_main("Starting NON-BLOCKING thread cleanup");
    g_threadShutdown = 1;  // Signal threads to shutdown
    
    // Cleanup socket thread
    if (g_hSocketThread) {
        DWORD result = WaitForSingleObject(g_hSocketThread, 500);  // Only 500ms wait
        if (result == WAIT_TIMEOUT) {
            log_main("Socket thread cleanup timeout - forcing termination");
            TerminateThread(g_hSocketThread, 0);
        }
        CloseHandle(g_hSocketThread);
        g_hSocketThread = NULL;
        log_main("Socket thread handle closed");
    }
    
    // Cleanup message processing thread
    if (g_hMessageThread) {
        DWORD result = WaitForSingleObject(g_hMessageThread, 500);  // Only 500ms wait
        if (result == WAIT_TIMEOUT) {
            log_main("Message thread cleanup timeout - forcing termination");
            TerminateThread(g_hMessageThread, 0);
        }
        CloseHandle(g_hMessageThread);
        g_hMessageThread = NULL;
        log_main("Message processing thread handle closed");
    }
    
    // Cleanup message queues
    cleanup_message_queue(&g_MainToSocket);
    cleanup_message_queue(&g_SocketToMain);
    log_main("Message queues cleaned up");
}

// Simple log function for main process
void log_main(const char* message) {
    FILE* logFile = fopen("D:\\leditapi\\ledit_python\\main_process_log.txt", "a");
    if (logFile) {
        fprintf(logFile, "[MAIN] %s\n", message);
        fclose(logFile);
    }
}

// Simple log function for socket child process
void log_socket(const char* message) {
    FILE* logFile = fopen("D:\\leditapi\\ledit_python\\socket_process_log.txt", "a");
    if (logFile) {
        fprintf(logFile, "[SOCKET] %s\n", message);
        fclose(logFile);
    }
}


void MyMacro(void) {
    // 测试宏 - GUI安全版本
    LDialog_MsgBox("Hello, world!");
    LStatusBar_SetMsg("MyMacro executed");
    log_main("MyMacro function called");
}

// Note: Old thread implementation has been replaced by pure message queue architecture above

// Timer callback function - executes in main thread context
void CALLBACK MainThreadTimerProc(HWND hwnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime) {
    // This function runs in the main thread context
    if (g_transistorPending) {
        log_main("Timer detected g_transistorPending flag - executing TransistorMacro in main thread");
        g_transistorPending = 0;  // Reset flag first
        
        try {
            TransistorMacro();  // Execute in main thread - should be completely safe
            log_main("TransistorMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing TransistorMacro via main thread timer");
        }
    }
    
    if (g_enhancedTransistorPending) {
        log_main("Timer detected g_enhancedTransistorPending flag - executing enhanced_transistor_macro in main thread");
        g_enhancedTransistorPending = 0;  // Reset flag first
        
        try {
            enhanced_transistor_macro(g_enhancedTransistorParams);  // Execute in main thread with saved parameters
            log_main("enhanced_transistor_macro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing enhanced_transistor_macro via main thread timer");
        }
    }

    if (g_memristorPending) {
        log_main("Timer detected g_memristorPending flag - executing memristor_macro in main thread");
        g_memristorPending = 0;  // Reset flag first
        
        try {
            memristor_macro(g_memristorParams);  // Execute in main thread with saved parameters
            log_main("memristor_macro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing memristor_macro via main thread timer");
        }
    }

    if (g_polygonPending) {
        log_main("Timer detected g_polygonPending flag - executing PolygonMacro in main thread");
        g_polygonPending = 0;  // Reset flag first
        
        try {
            PolygonMacro(g_polygonParams);  // 调用新的多边形绘制函数
            log_main("PolygonMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing PolygonMacro via main thread timer");
        }
    }
    
    if (g_pathPending) {
        log_main("Timer detected g_pathPending flag - executing PathMacro in main thread");
        g_pathPending = 0;  // Reset flag first
        
        try {
            PathMacro(g_pathParams); 
            log_main("PathMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing PathMacro via main thread timer");
        }
    }

    if (g_textPending) {
        log_main("Timer detected g_textPending flag - executing TextMacro in main thread");
        g_textPending = 0;  // Reset flag first
        
        try {
            TextMacro(g_textParams); 
            log_main("TextMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing TextMacro via main thread timer");
        }
    }

    if (g_circlePending) {
        log_main("Timer detected g_circlePending flag - executing CiecleMacro in main thread");
        g_circlePending = 0;  // Reset flag first
        
        try {
            CircleMacro(g_circleParams); 
            log_main("CircleMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing CircleMacro via main thread timer");
        }
    }

    if (g_piewedgePending) {
        log_main("Timer detected g_piewedgePending flag - executing PieWedgeMacro in main thread");
        g_piewedgePending = 0;  // Reset flag first
        
        try {
            PieWedgeMacro(g_piewedgeParams); 
            log_main("PieWedgeMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing PieWedgeMacro via main thread timer");
        }
    }
    
    if (g_ringwedgePending) {
        log_main("Timer detected g_ringwedgePending flag - executing RingWedgeMacro in main thread");
        g_ringwedgePending = 0;  // Reset flag first
        
        try {
            RingWedgeMacro(g_ringwedgeParams);
            log_main("RingWedgeMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing RingWedgeMacro via main thread timer");
        }
    }

    if (g_boolexprPending) {
        log_main("Timer detected g_boolexprPending flag - executing BoolExprMacro in main thread");
        g_boolexprPending = 0;  // Reset flag first
        
        try {
            BoolExprMacro(g_boolexprParams); 
            log_main("BoolExprMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing BoolExprMacro via main thread timer");
        }
    }
    
    if (g_arrayPending) {
        log_main("Timer detected g_arrayPending flag - executing ArrayMacro in main thread");
        g_arrayPending = 0;  // Reset flag first
        
        try {
            ArrayMacro(g_arrayParams); 
            log_main("ArrayMacro executed successfully via main thread timer");
        } catch (...) {
            log_main("ERROR: Exception occurred while executing ArrayMacro via main thread timer");
        }
    }
}

// Start main thread timer
void start_main_thread_timer(void) {
    if (g_mainThreadTimerID == 0) {
        // Create a timer that checks every 100ms
        g_mainThreadTimerID = SetTimer(NULL, 0, 100, MainThreadTimerProc);
        if (g_mainThreadTimerID != 0) {
            log_main("Main thread timer started successfully - will check for pending transistor execution every 100ms");
        } else {
            log_main("ERROR: Failed to create main thread timer");
        }
    } else {
        log_main("Main thread timer already running");
    }
}

// Stop main thread timer
void stop_main_thread_timer(void) {
    if (g_mainThreadTimerID != 0) {
        KillTimer(NULL, g_mainThreadTimerID);
        g_mainThreadTimerID = 0;
        log_main("Main thread timer stopped");
    } else {
        log_main("Main thread timer was not running");
    }
}

int UPI_Entry_Point(void) {
    //LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "MyMacro", "MyMacro", NULL);
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "socket_main", "socket_main", NULL);
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "stop_socket_server", "stop_socket_server", NULL);
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "check_and_execute_transistor_macro", "check_and_execute_transistor_macro", NULL);
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "start_main_thread_timer", "start_main_thread_timer", NULL);
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "stop_main_thread_timer", "stop_main_thread_timer", NULL);
    
    // 绑定新的增强功能到菜单
    LMacro_BindToMenuAndHotKey_v9_30("Tools", NULL, "initialize_macro_queue_system", "initialize_macro_queue_system", NULL);
    
    return 1;
}
