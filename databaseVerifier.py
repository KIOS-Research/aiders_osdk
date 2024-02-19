#!/usr/bin/env
# -*- coding: utf-8 -*-
# author: Christos Georgiades
# contact: chr.georgiades@gmail.com
# date: 28-12-2022


import sys
import sqlite3

print ('Number of arguments:', len(sys.argv), 'arguments.')
print ('Argument List:', str(sys.argv))



# Create DB instance
dbInstance1 = sys.argv[1]
dbInstance2 = sys.argv[2]

print ('dbInstance1:', str(dbInstance1))
print ('dbInstance2:', str(dbInstance2))


db1 = sqlite3.connect(dbInstance1, check_same_thread=False)
dbCursor1 = db1.cursor()

db2 = sqlite3.connect(dbInstance2, check_same_thread=False)
dbCursor2 = db2.cursor()

def getTables(cursor):
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    return cursor.fetchall()


def compareTables(tableName):
    dbCursor1.execute("select * from " + tableName)
    results1 = dbCursor1.fetchall()
    dbCursor2.execute("select * from " + tableName)
    results2 = dbCursor2.fetchall()
    count1 =  len(results1)
    count2 =  len(results2)
    
    print('count1:', count1)
    print('count2:', count2)
    
    for entry in results1:
        if entry in results2:
            #print('entry exists')
            results1.remove(entry)
            results2.remove(entry)
        else:
            print('entry does not exist in db2', entry[0], entry[1])
            
    for entry in results2:
        if entry in results1:
            #print('entry exists')
            results1.remove(entry)
            results2.remove(entry)
        else:
            print('entry does not exist in db1', entry[0], entry[1])
            
    print('final count1:', len(results1))
    print('final count2:', len(results2))
    
    print('\ndb1:')
    
    for entry in results1:
        print(entry[0], entry[1])
        
    print('\n\n=====\n\n')
    print('db2:')
            
    for entry in results2:
        print(entry[0], entry[1])
    


dbTables1 = getTables(dbCursor1)
dbTables2 = getTables(dbCursor2)

print(dbTables1)
print(dbTables2)

#Serialize table names
print('Number of dbTables1:', len(dbTables1))
for index in range(len(dbTables1)):
    tableName = str(dbTables1[index][0].encode('utf-8').split(" ")[0])
    dbTables1[index] = tableName
    print('\t' + tableName)

print('Number of dbTables2:', len(dbTables1))
for index in range(len(dbTables2)):
    tableName = str(dbTables2[index][0].encode('utf-8').split(" ")[0])
    dbTables2[index] = tableName
    print('\t' + tableName)



for table in dbTables1:    
    print(table)
    
    if table in  dbTables2:
        print('tableName:', table, 'exists in both databases')
        compareTables(table)
    else:
        print('Not found')