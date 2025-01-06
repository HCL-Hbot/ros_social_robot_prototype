# POC eyes with Electron

## installation

```bash
mkdr project_dr #Voor mij
cd project_dr #Voor mij
npm init -y # Voor mij initialiseer project met npm
npm install --save-dev electron #Install electron 
npm install websocket #Install websocket package for client side
npm install ws #Install websocket for server side
npm install sass --save-dev #Using SCSS for styling, package which can compile to CSS
npm install concurrently --save-dev #Using to run multiple commands in one line (npm start and npm sass)
```


## Possible future improvements
When touch functionality will integrated in another project. Don't forget disable zoom in and scrolling with javascript (for touch device)! Because zooming in a eye is prorabably not a desired functionalty.
No code is written for enabling/disabling touch. I don't know what the default is. This was not in my scope. Just reminding some logic needed to be added for this.