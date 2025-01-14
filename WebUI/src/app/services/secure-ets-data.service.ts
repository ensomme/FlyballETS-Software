import { Injectable } from '@angular/core';
import { ConfigArray } from '../interfaces/config-array';
import { Observable, Observer, Subject, BehaviorSubject } from 'rxjs';
import { share } from 'rxjs/operators';
import { WebsocketDataRequest } from '../interfaces/websocket-data-request';

@Injectable()
export class SecureEtsDataService {
   ETS_URL:string = "ws://"+ window.location.host +"/wsa";
   wsObservable:Observable<any>;
   wsObserver:Observer<any>;
   private ws;
   public dataStream:BehaviorSubject<any>;
   isConnected:boolean;
   sessionEnded:boolean;
   
   private _isAuthenticated = new BehaviorSubject<boolean>(true);
   public isAuthenticated: Observable<boolean> = this._isAuthenticated.asObservable();

   public setAuthenticated(authenticated:boolean): void {
      console.log('authenticated: ' + authenticated);
      this._isAuthenticated.next(authenticated);
   }

   constructor() {
      this.initializeWebSocket();
   }

   initializeWebSocket() {
      console.log("Reconnecting to wsa");
      this.isConnected = false;
      this.wsObservable = Observable.create((observer) => {
         this.ws = new WebSocket(this.ETS_URL);
         console.log('created wsa');
         this.ws.onopen = (e) => {
            this.isConnected = true;
            console.log("Connected wsa!");
         };

         this.ws.onclose = (e) => {
            if (e.wasClean) {
               observer.complete();
            } else {
               observer.error(e);
            }
            this.isConnected = false;
            console.log('onClose');
         };
      
         this.ws.onerror = (e) => {
            observer.error(e);
            this.isConnected = false;
            console.log('onError');
         }
      
         this.ws.onmessage = (e) => {
            observer.next(JSON.parse(e.data));
         }
      
         return () => {
            console.log("Connection closed gracefully");
            this.ws.close();
            this.isConnected = false;
         };
      }).pipe(share());
      console.log('observer created');

      this.wsObserver = {
         next: (data: Object) => {
            if (this.ws.readyState === WebSocket.OPEN) {
               console.log(data)
               this.ws.send(JSON.stringify(data));
            }
         },
         error: (err) => {
            console.log("Error sending data:");
            console.log(err);
         },
         complete: () => {

         }
      }

      this.dataStream = Subject.create(this.wsObserver, this.wsObservable);
   }

   sendConfig(config:ConfigArray) {
      this.dataStream.next(config);
   }

   getData(dataRequest:WebsocketDataRequest) {
      this.dataStream.next(dataRequest);
   }
}