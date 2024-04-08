
// Localhost:8080 ist zum testen wenn die API lokal läuft
const API_ADDRESS = "http://localhost:8080";

// mit dier IP lässt sich die API im Uninetzwerk oder mit VPN erreichen
//const API_ADDRESS = "http://10.5.20.128:8080";

const AUTH_TOKEN_LOCATION = "llmApi.authToken"

// Dieser Endpunkt liefert den aktuell eingeloggten User
const checkForAccess = async() => {
    const tokenInStorage = localStorage.getItem(AUTH_TOKEN_LOCATION);
    if(!tokenInStorage)
        return false

    response = await fetch(API_ADDRESS + '/users/get/current', {
        method: 'GET',
        headers: {
            'Authorization': `Bearer ${tokenInStorage}`,
        },
    });
    return response.status == 200

}

// Mit diesem Endpunkt kann man sich einloggen
// johndoe ist momentan noch unser Superuser
// TODO: Login über Dialogfenster mit userinput
const login =  async() =>{
    response = await fetch(API_ADDRESS + '/login', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
        body: new URLSearchParams({
            username: 'johndoe',
            password: 'secret'
          })
    });

    const jsonAnswer = await response.json();
    localStorage.setItem(AUTH_TOKEN_LOCATION, jsonAnswer["access_token"])
}

// Hier kann ein neuer Chat erstellt werden der dem aktuellen user zugeordnet wird
const getNewChat = async(name) => {
    const authToken = localStorage.getItem(AUTH_TOKEN_LOCATION);
    if(!authToken) return;

    response = await fetch(API_ADDRESS + '/chats/new', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${authToken}`,
        },
        body: JSON.stringify({
            name
        })
    });

    const jsonAnswer = await response.json();
    return jsonAnswer["id"];
}

// Endpunkt um eine Anwort zu erhalten
// ChatName ist optional. Wird der Request ohne namen abgeschickt wird die Nachricht automatisch dem Default Chat des user zugeornet
// Dieser Endpunkt funktioniert ohne streaming, also wird nur die fertige Antwort zurückgegeben
const requestAnswer = async(question, chatName = undefined) => {
    const authToken = localStorage.getItem(AUTH_TOKEN_LOCATION);
    if(!authToken) return;

    response = await fetch(API_ADDRESS + '/messages/new/answer', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${authToken}`,
        },
        body: JSON.stringify({
            chat_name: chatName,
            content: question
        })
    });

    const jsonAnswer = await response.json();
    return jsonAnswer["content"];
}

// Endpunkt um eine Anwort zu erhalten
// ChatName ist optional. Wird der Request ohne namen abgeschickt wird die Nachricht automatisch dem Default Chat des user zugeornet
// Dieser Endpunkt funktioniert mit streaming, also erhält man einen stream als Antwort, der immer wieder Teile der Antwort zurückgibt
const requestAnswerStream = async(question, chatName = undefined) => {
    const authToken = localStorage.getItem(AUTH_TOKEN_LOCATION);
    if(!authToken) return;

    const response = await fetch(API_ADDRESS + '/messages/new/stream_answer',{
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${authToken}`,
        },
        body: JSON.stringify({
            chat_name: chatName,
            content: question
        })
    })
    return response.body
}


// Die nächsten 3 Funktionen lesen den Stream aus und liefern Strings zurück
async function *parseStream(stream) {
    for await (const line of readLines(stream.getReader())) {
        const trimmedLine = line.trim();
        yield trimmedLine
    }
}

async function *readLines(reader) {
    const textDecoder = new TextDecoder('utf-8');
    for await (const chunk of readChunks(reader)) {
        const chunkText = textDecoder.decode(chunk)
        yield chunkText;


    }
}

function readChunks(reader) {
    return {
        async* [Symbol.asyncIterator]() {
            let readResult = await reader.read();
            while (!readResult.done) {
                yield readResult.value;
                readResult = await reader.read();
            }
        },
    };
}


// Teilt die Teile des Streams in Worte
// Der Stream liefert in Chunks immer ganze Worte zurück (ein oder mehrere)
// § Representiert Leerzeichen die ersetzt werden müssen
// onNewWord ist eine Funktion die definiert wie die Wörter verarbeitet werden müssen
const handleAnswerStream = async (stream, onNewWord) => {
    for await (chunk of parseStream(stream)) {
        chunk.split("§").forEach((word) => {
            if(word != "")
                onNewWord(word);
        })
    }
}



var chatMessages = document.getElementById('chatMessages');
var chatInput = document.getElementById('chatInput');
var chatForm = document.getElementById('chatForm');
var myModal = document.getElementById('chatModal')


var robotSvg = '<svg xmlns="http://www.w3.org/2000/svg" height="45" viewBox="0 -960 960 960" width="24"><path d="M160-360q-50 0-85-35t-35-85q0-50 35-85t85-35v-80q0-33 23.5-56.5T240-760h120q0-50 35-85t85-35q50 0 85 35t35 85h120q33 0 56.5 23.5T800-680v80q50 0 85 35t35 85q0 50-35 85t-85 35v160q0 33-23.5 56.5T720-120H240q-33 0-56.5-23.5T160-200v-160Zm200-80q25 0 42.5-17.5T420-500q0-25-17.5-42.5T360-560q-25 0-42.5 17.5T300-500q0 25 17.5 42.5T360-440Zm240 0q25 0 42.5-17.5T660-500q0-25-17.5-42.5T600-560q-25 0-42.5 17.5T540-500q0 25 17.5 42.5T600-440ZM320-280h320v-80H320v80Zm-80 80h480v-480H240v480Zm240-240Z"/></svg>'

var avatarSvg = '<svg xmlns="http://www.w3.org/2000/svg" height="45" viewBox="0 -960 960 960" width="24"><path d="M234-276q51-39 114-61.5T480-360q69 0 132 22.5T726-276q35-41 54.5-93T800-480q0-133-93.5-226.5T480-800q-133 0-226.5 93.5T160-480q0 59 19.5 111t54.5 93Zm246-164q-59 0-99.5-40.5T340-580q0-59 40.5-99.5T480-720q59 0 99.5 40.5T620-580q0 59-40.5 99.5T480-440Zm0 360q-83 0-156-31.5T197-197q-54-54-85.5-127T80-480q0-83 31.5-156T197-763q54-54 127-85.5T480-880q83 0 156 31.5T763-763q54 54 85.5 127T880-480q0 83-31.5 156T763-197q-54 54-127 85.5T480-80Zm0-80q53 0 100-15.5t86-44.5q-39-29-86-44.5T480-280q-53 0-100 15.5T294-220q39 29 86 44.5T480-160Zm0-360q26 0 43-17t17-43q0-26-17-43t-43-17q-26 0-43 17t-17 43q0 26 17 43t43 17Zm0-60Zm0 360Z"/></svg>'

//Hier kann nun eine id übergeben werden um die Nachricht später dynamisch zu füllen
const getMessage = (message, isServer, id) => `<div class="d-flex flex-row${isServer ? "" : "-reverse" } justify-content-start m-3"><div style="width: 45px; height: 100%;">${isServer ? robotSvg : avatarSvg}</div><div class="p-3 mx-3" style="border-radius: 15px; background-color: rgba(57, 192, 237,.2); max-width: 550px; word-break: break-word; white-space: pre-wrap;"><p data-id=${id} class="small mb-1 text-dark">${message}</p></div></div>`;

let nextId = 0;
const getId = (prefix) => {
    nextId += 1;
    return `${prefix}-${nextId}`
}

let chatName = "Default";
let firstMessage = false;

chatForm.onsubmit = async function() {

    const hasAccess = await checkForAccess();
    console.log(hasAccess)
    if(!hasAccess) await login();

    if(firstMessage)
        await getNewChat(chatName);
    firstMessage = false;



    var value = chatInput.value

    if (value == "") {
        return
    }


    chatMessages.innerHTML += getMessage(value, false, getId("Human"));

    const answerId = getId("AI");
    chatMessages.innerHTML += getMessage("", true, answerId);
    const answerMessage = document.querySelector(`[data-id="${answerId}"]`);
    const answerStream = await requestAnswerStream(value);

    //Füllt die Antwort
    const addWordToMessage = (word) => {
        answerMessage.innerHTML += word + " ";
    }

    await handleAnswerStream(answerStream, addWordToMessage)

    // clear input
    chatInput.value = '';
    chatMessages.scrollTo(0, chatMessages.scrollHeight);
}

function submitOnEnter(event) {
    if (event.which === 13 && !event.shiftKey) {
        if (!event.repeat) {
            const newEvent = new Event("submit", {cancelable: true});
            event.target.form.dispatchEvent(newEvent);
        }

        event.preventDefault(); // Prevents the addition of a new line in the text field
    }
}

chatInput.addEventListener("keydown", submitOnEnter);
myModal.addEventListener('shown.bs.modal', () => {
  chatInput.focus()
})
