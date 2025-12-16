package main

import (
    "bufio"
    "fmt"
    "net/http"
    "os"
    "time"
    "strconv"
)

func makeRequest(url string, beforeSleepSec int, afterSleepSec int) {
    time.Sleep(time.Duration(beforeSleepSec) * time.Second)

    resp, err := http.Get(url)
    if err != nil {
        panic(err)
    }
    defer resp.Body.Close()
    fmt.Println("Response status:", resp.Status)

    scanner := bufio.NewScanner(resp.Body)
    for i := 0; scanner.Scan() && i < 5; i++ {
        fmt.Println(scanner.Text())
    }

    if err := scanner.Err(); err != nil {
        panic(err)
    }

    time.Sleep(time.Duration(afterSleepSec) * time.Second)
}

func main() {
    if (len(os.Args) < 2) {
        fmt.Println("Usage: httpclient <url> [count] [before_sleep_sec] [after_sleep_sec]")
        os.Exit(-1);
    }
    url := os.Args[1]
   
    count := 1 
    beforeSleepSec := 0
    afterSleepSec := 0

    if (len(os.Args) >= 3) {
        count, _ = strconv.Atoi(os.Args[2])
    }

    if (len(os.Args) >= 4) {
        beforeSleepSec, _ = strconv.Atoi(os.Args[3])
    }

    if (len(os.Args) >= 5) {
        afterSleepSec, _ = strconv.Atoi(os.Args[4])
    }
   
    for i := 0; i < count; i++ {
        makeRequest(url, beforeSleepSec, afterSleepSec)
    }
}
