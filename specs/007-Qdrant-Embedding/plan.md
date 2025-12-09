## Step 1 – Setup
- Load environment variables using `python-dotenv`.
- Initialize Cohere client.
- Initialize Qdrant client.

## Step 2 – Sitemap Parsing
- Fetch XML sitemap from `sitemap_url`.
- Parse XML using `xml.etree.ElementTree`.
- Extract URLs from `<loc>` tags.

## Step 3 – Text Extraction
- Fetch HTML content from each URL.
- Extract clean text using `trafilatura`.
- Log warnings if text extraction fails.

## Step 4 – Text Chunking
- Split text into ≤4000 character chunks.
- Ensure splitting occurs at sentence boundaries when possible.

## Step 5 – Embedding Generation
- Use Cohere API to generate embeddings for each chunk.
- Ensure embeddings are consistent in dimensionality (1024).

## Step 6 – Qdrant Storage
- Check if Qdrant collection exists; create if missing.
- Insert chunks as `PointStruct` with metadata (`url`, `chunk_id`, `text`).

## Step 7 – Logging & Monitoring
- Print status updates for each URL and chunk.
- Summarize total chunks stored at the end.

## Step 8 – Execution
- Run `ingest_book()` to perform full ingestion.
- Designed to run once per dataset.
