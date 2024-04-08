import express from "express";
import path from "path";
import cors from "cors";

import { fileURLToPath } from 'url';
import { dirname } from 'path';


// Ich glaube ein kleiner WebServer ist nötig a
// Ohne die Einstellungen mit Cors blockiert der Browser Anfragen an die API

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();

app.use(cors())
app.options('*', cors());
app.use(express.static(path.join(__dirname, '_build/html')));

app.listen(3000, () => {
  console.log('Server is running on port 3000');
});
