//
// Created by acer on 25-8-31.
//

#ifndef LOGGING_H
#define LOGGING_H

#ifndef MAIN_PROJECT__LOGGINGH_H_
#define MAIN_PROJECT__LOGGINGH_H_

#define __FILENAME__ (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOGI(fmt, ...) fprintf(stderr, "\033[1;32m[INFO]  [%s:%d]: " fmt "\033[0m\n", __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOGW(fmt, ...) fprintf(stderr, "\033[1;33m[WARN]  [%s:%d]: " fmt "\033[0m\n", __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOGD(fmt, ...) fprintf(stderr, "\033[1;34m[DEBUG] [%s:%d]: " fmt "\033[0m\n", __FILENAME__, __LINE__, ##__VA_ARGS__)
#define LOGE(fmt, ...) fprintf(stderr, "\033[1;31m[ERROR] [%s:%d]: " fmt "\033[0m\n", __FILENAME__, __LINE__, ##__VA_ARGS__)

#endif //MAIN_PROJECT__LOGGINGH_H_

#endif //LOGGING_H
